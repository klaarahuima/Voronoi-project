#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
#include <cmath>
#include <random>
#include <string>
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <list>
#include "lbfgs.c"
#include <cassert>
#include <chrono>

double sqr(double x) {return x*x;};

static std::default_random_engine engine (10) ; // random seed = 10
static std::uniform_real_distribution<double> uniform (0,1);


class Vector {
    public:
        explicit Vector(double x = 0, double y = 0, double z = 0) {
            data[0] = x;
            data[1] = y;
            data[2] = z;
        }
        double norm2() const {
            return data[0] * data[0] + data[1] * data[1] + data[2] * data[2];
        }
        double norm() const {
            return sqrt(norm2());
        }
        void normalize() {
            double n = norm();
            data[0] /= n;
            data[1] /= n;
            data[2] /= n;
        }
        double operator[](int i) const { return data[i]; };
        double& operator[](int i) { return data[i]; };
        double data[3];
    };
     
    Vector operator+(const Vector& a, const Vector& b) {
        return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
    }
    Vector operator-(const Vector& a, const Vector& b) {
        return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    }
    Vector operator*(const double a, const Vector& b) {
        return Vector(a*b[0], a*b[1], a*b[2]);
    }
    Vector operator*(const Vector& a, const double b) {
        return Vector(a[0]*b, a[1]*b, a[2]*b);
    }
    Vector operator/(const Vector& a, const double b) {
        return Vector(a[0] / b, a[1] / b, a[2] / b);
    }
    double dot(const Vector& a, const Vector& b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }
    Vector cross(const Vector& a, const Vector& b) {
        return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
    }


class Polygon {  
public:
    std::vector<Vector> vertices;

    double area() {
        double s = 0;
        for (int i = 0; i < vertices.size(); i++) {
            int ip = (i==vertices.size()-1) ? 0 : (i+1);
            s += vertices[i][0]*vertices[ip][1] - vertices[ip][0] * vertices[i][1];

        }
        s = std::abs(s) / 2.0;
        return s ;
    }

    double integral_sqr_dist(const Vector& Pi) {
        // decompose polygon into triangles
        // n-2 triangles if n is number of vertices
        double s = 0;
        if (vertices.size() < 3) {
            return 0;
        }
        for (int t = 1;t < vertices.size()-2; t++) {
            Vector c[3] = {vertices[0], vertices[t], vertices[t+1]}; // triangle with 3 vertices
            // structure holds for every t : will define n-2 triangles that split the polygon exactly

            double integralT = 0;
            for (int k = 0; k < 3; k++) {
                for (int l = k; l < 3; l++) {
                    integralT += dot(c[k]-Pi, c[l]-Pi);

                }
            }
            double areaT = 0.5*std::abs((c[1][0]-c[0][0])*(c[2][1]-c[0][1]) - (c[1][1]-c[0][1])*(c[2][0]-c[0][0]));
            s += integralT * areaT / 6.0;

        }
    return s;
    }
};  


class Voronoi {
    public:
    Voronoi() {};

    // returns a clipped polygon
    Polygon clip_by_bisector(const Polygon& V, const Vector& P0, const Vector& Pi, double w0, double wi) {
        Polygon  result;
        for (int i = 0; i<V.vertices.size(); i++) {


            const Vector &A = V.vertices[(i == 0)? V.vertices.size()-1:i-1]; // previous vertex
            const Vector &B = V.vertices[i];

            bool A_inside = (A - P0).norm2() -w0 <= (A - Pi).norm2() -wi;
            bool B_inside = (B - P0).norm2() -w0 <= (B - Pi).norm2() -wi;

        
            Vector v = Pi - P0;
            Vector M = (P0 + Pi) / 2 + ((w0 - wi) / (2 * v.norm2())) * v; // from lecture notes
            

            if (B_inside) { // algorithm in lecture notes, defined by P0 and Pi

                if (!A_inside) {

                    // unweighted version
                    //Vector M = (P0 + Pi) * 0.5;
                    //double t = dot(M-A, Pi - P0) / dot(B-A, Pi - P0);
                    //Vector P = A + t*(B-A);
                   
                    double t = dot(M - A, v)/dot(B - A, v);
                    Vector P = A + t*(B-A);
                    result.vertices.push_back(P); // P defined as in lecture
                
                }
            
                result.vertices.push_back(B);

            } else {

                if (A_inside) {

                    // unweighted version
                    //Vector M = (P0 + Pi) * 0.5;
                    //double t = dot(M-A, Pi - P0) / dot(B-A, Pi - P0);
                    //Vector P = A + t*(B-A);
                    
                    double t = dot(M - A, v)/dot(B - A, v);
                    Vector P = A + t*(B-A);
                    result.vertices.push_back(P);
        
                }
            }

        }
        return result;
    }

    void compute() {

        Polygon square;
        square.vertices.push_back(Vector(0, 0));
        square.vertices.push_back(Vector(1, 0));
        square.vertices.push_back(Vector(1, 1));
        square.vertices.push_back(Vector(0, 1));

        cells.resize(points.size());        

        for (int i = 0; i < points.size(); i++) {

            Polygon V = square;

            for (int j = 0; j < points.size(); j++) {
                if (i == j) continue;

                double w0 = weights[i];
                double wi = weights[j];

                V = clip_by_bisector(V, points[i], points[j], w0, wi);

            }
            cells[i] = V;
        }
    }
    std::vector<Vector> points;
    std::vector<Polygon> cells;
    std::vector<double> weights;

};
 
// saves a static svg file. The polygon vertices are supposed to be in the range [0..1], and a canvas of size 1000x1000 is created
void save_svg(const std::vector<Polygon> &polygons, const std::vector<Vector> &seeds, std::string filename, std::string fillcol = "none") {
        FILE* f = fopen(filename.c_str(), "w+"); 
        fprintf(f, "<svg xmlns = \"http://www.w3.org/2000/svg\" width = \"1000\" height = \"1000\">\n");

        for (int i=0; i<polygons.size(); i++) {
            fprintf(f, "<g>\n");
            fprintf(f, "<polygon points = \""); 
            for (int j = 0; j < polygons[i].vertices.size(); j++) {
                fprintf(f, "%3.3f, %3.3f ", (polygons[i].vertices[j][0] * 1000), (1000 - polygons[i].vertices[j][1] * 1000));
            }
            fprintf(f, "\"\nfill = \"%s\" stroke = \"black\"/>\n", fillcol.c_str());
            fprintf(f, "</g>\n");
        }
        for (const auto& seed : seeds) {
        double cx = seed[0] * 1000;
        double cy = 1000 - seed[1] * 1000;
        fprintf(f, "<circle cx=\"%3.3f\" cy=\"%3.3f\" r=\"3\" fill=\"red\" />\n", cx, cy);
        }
        fprintf(f, "</svg>\n");
        fclose(f);
    }
 
 
// Adds one frame of an animated svg file. frameid is the frame number (between 0 and nbframes-1).
// polygons is a list of polygons, describing the current frame.
// The polygon vertices are supposed to be in the range [0..1], and a canvas of size 1000x1000 is created
void save_svg_animated(const std::vector<Polygon> &polygons, std::string filename, int frameid, int nbframes) {
        FILE* f;
        if (frameid == 0) {
            f = fopen(filename.c_str(), "w+");
            fprintf(f, "<svg xmlns = \"http://www.w3.org/2000/svg\" width = \"1000\" height = \"1000\">\n");
            fprintf(f, "<g>\n");
        } else {
            f = fopen(filename.c_str(), "a+");
        }
        fprintf(f, "<g>\n");
        for (int i = 0; i < polygons.size(); i++) {
            fprintf(f, "<polygon points = \""); 
            for (int j = 0; j < polygons[i].vertices.size(); j++) {
                fprintf(f, "%3.3f, %3.3f ", (polygons[i].vertices[j][0] * 1000), (1000-polygons[i].vertices[j][1] * 1000));
            }
            fprintf(f, "\"\nfill = \"none\" stroke = \"black\"/>\n");
        }
        fprintf(f, "<animate\n");
        fprintf(f, "    id = \"frame%u\"\n", frameid);
        fprintf(f, "    attributeName = \"display\"\n");
        fprintf(f, "    values = \"");
        for (int j = 0; j < nbframes; j++) {
            if (frameid == j) {
                fprintf(f, "inline");
            } else {
                fprintf(f, "none");
            }
            fprintf(f, ";");
        }
        fprintf(f, "none\"\n    keyTimes = \"");
        for (int j = 0; j < nbframes; j++) {
            fprintf(f, "%2.3f", j / (double)(nbframes));
            fprintf(f, ";");
        }
        fprintf(f, "1\"\n   dur = \"5s\"\n");
        fprintf(f, "    begin = \"0s\"\n");
        fprintf(f, "    repeatCount = \"indefinite\"/>\n");
        fprintf(f, "</g>\n");
        if (frameid == nbframes - 1) {
            fprintf(f, "</g>\n");
            fprintf(f, "</svg>\n");
        }
        fclose(f);
    }

static lbfgsfloatval_t evaluate(void* , const lbfgsfloatval_t*, lbfgsfloatval_t*, const int, const lbfgsfloatval_t);

class OptimalTransport {
    public:
    OptimalTransport(){};

    void optimize() {

        // This code is from the lecture

        int N = vor.weights.size();
        lbfgsfloatval_t fx;
        std::vector<double> weights(N, 0);
        memcpy(&weights[0], &vor.weights[0], N*sizeof(weights[0]));
        assert(vor.weights.size() == N);

        lbfgs_parameter_t param;
        lbfgs_parameter_init(&param);

        int ret = lbfgs(N, &weights[0], &fx, evaluate, NULL, (void*)this, &param);
        std::cout << "L-BFGS terminated with status " << ret << std::endl;

        memcpy(&vor.weights[0], &weights[0], N*sizeof(weights[0]));
        vor.compute();
    }

    Voronoi vor;
};

static lbfgsfloatval_t evaluate(
        void *instance,
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t step
        )
    {
        // This code is from the lecture
        OptimalTransport* ot = (OptimalTransport*)(instance);

        int N = ot->vor.weights.size();
        memcpy(&ot->vor.weights[0], x, N*sizeof(x[0]));
        ot->vor.compute();

        // Define target areas below

        std::vector<Vector> sites = ot->vor.points;
        std::vector<double> target_areas(sites.size());

        Vector center(0.5, 0.5, 0.);
        double variance = 0.08;
        double tot = 0.;

        // computing target areas as shown in lecture notes
        for (int i = 0; i < sites.size(); i++) {
            Vector dist = sites[i] - center;
            double tmp = dist.norm2();
            target_areas[i] = std::exp(-tmp / variance);
            tot += target_areas[i];
        }

        // normalizing target areas
        for (int i = 0; i < sites.size(); i++) {
            target_areas[i] = target_areas[i] / tot;
        }

        int i;
        lbfgsfloatval_t fx = 0.;
        for (int i = 0; i < n; i++){ 
                double current_area = ot->vor.cells[i].area();
                g[i] = -(target_areas[i] - current_area);
                fx += ot->vor.cells[i].integral_sqr_dist(ot->vor.points[i]) - x[i]*(current_area - target_areas[i]);
                //std::cout << "Cell " << i << ": Area = " << current_area << ", Target = " << 1./n << ", Gradient = " << g[i] << std::endl;
        }
        return -fx;
    }


int main() {

    int N = 1000;
    Voronoi vor;

    for (int i = 0; i < N; i++) { // creating random points
        vor.points.push_back(Vector(uniform(engine), uniform(engine), 0.0));
        vor.weights.push_back(0.);
    }

    vor.compute();
    save_svg(vor.cells, vor.points, "test.svg", "none");
   
    std::cout << "OPTIMAL TRANSPORT NOW ------------------------------------" << std::endl;
    
    auto start = std::chrono::high_resolution_clock::now();

    OptimalTransport ot;
    ot.vor = vor;
    ot.optimize();

    save_svg(ot.vor.cells, ot.vor.points, "voronoi.svg", "none");


    auto end2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start);
    std::cout << "Elapsed time: " << duration2.count() << " ms\n";

    return 0;


}