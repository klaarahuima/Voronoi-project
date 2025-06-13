#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
#include <cmath>

#include <string>
#include <iostream>
#include <stdio.h>
#include <algorithm>
 
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
 
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

double sqr(double x) { return x*x;};
 
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

class Ray {
public:
    Ray(const Vector& O, const Vector &u) : O(O), u(u) {};
    // O is the origin, u is the directional vector
    Vector O,u;

};
/*
class Object {
    public:
    virtual bool intersect(const Ray& r, Vector& P, Vector& N, double& t) = 0;
};
 */
class Sphere {
public:
    //Sphere(const Vector& C, double R, Vector alb, bool mirror, bool transparent) : C(C), R(R), alb(alb), mirror(mirror), transparent(transparent) {};
    Sphere(const Vector& C, double R, Vector alb, bool mirror, bool transparent) : C(C), R(R) {
                this->alb = alb;
                this->mirror = mirror;
                this->transparent = transparent;
            }
    Vector C; // center
    double R; // radius
    Vector alb; // albedo, color property
    bool mirror;
    bool transparent;
    
    bool intersect(const Ray& r, Vector &P, Vector &N, double &t) { // P : point of intersection, N : normal vector (reflected ray)
        // determines whether the ray intersects with the sphere and updates P and N with relevant info
        //double delta = sqr(dot(r.u, r.O - C)) - ((r.O - C).norm2() - R * R); // definition in lecture slides
        Vector OC = r.O - C;
        double a = dot(r.u, r.u);           
        double b = 2 * dot(OC, r.u);
        double c = dot(OC, OC) - R * R;

        double delta = b * b - 4 * a * c;
        
        if (delta < 0) {
            return false;
        } else {
        double x = dot(r.u, C - r.O);
        double t1 = x - sqrt(delta);
        double t2 = x + sqrt(delta);
        if (t2 < 0) {return false;} // sphere is behind the ray
        if (t1 >= 0) { // if t1 negative then ray starts from in the middle of the sphere (direction negative)
            t = t1;
        } else {t = t2;}
        P = r.O + t * r.u; // extend ray from origin to surface of sphere (multiply unit vector by t)
        N = P - C; // directional vector from point of contact to center of sphere, gives normal
        N.normalize();
        return true;
        }
        return (delta >=0);
    /*bool intersect(const Ray& r, Vector& P, Vector& N, double& t) {
        Vector OC = r.O - C;
        double b = dot(r.u, OC);
        double c = OC.norm2() - R*R;
        double delta = b*b - c;

        if (delta < 0) return false;

        double sqrt_delta = sqrt(delta);
        double t1 = -b - sqrt_delta;
        double t2 = -b + sqrt_delta;

        if (t2 < 0) return false;

        if (t1 > 0) t = t1;
        else t = t2;

        P = r.O + t * r.u;
        N = P - C;
        N.normalize();
        return true;
    }*/
    }
};

 
class Scene {
    public:
        Scene() {};
        std::vector<Sphere> objects;
        void add(const Sphere& s){objects.push_back(s);}
        Vector L; // light source point coordinates
        double I;

        bool intersect(const Ray& r, Vector& P, Vector& N, double& t, int& id) {
            t = 1e30;
            bool intersected = false;
            Vector localP, localN;
            double localt;
            for (int i = 0; i < objects.size(); i++) {
                if (objects[i].intersect(r, localP, localN, localt)) {
                    if (localt < t) {
                        t = localt;
                        P = localP;
                        N = localN;
                        id = i;
                        intersected = true;
                    }
                }
            }
            return intersected;
        }

        Vector getColor(const Ray r, int bounce_number) {

            if (bounce_number <= 0) return Vector(0,0,0);

            Vector P,N;
            double t;
            int object_id;

            if (!intersect(r, P, N, t, object_id)) {
                return Vector(0, 0, 0);
            }

            if(objects[object_id].mirror) {
                Vector reflection_dir = r.u - 2*dot(r.u, N)*N;
                Ray mirrorR(P + 0.001*N, reflection_dir);
                return getColor(mirrorR, bounce_number - 1);
            }

            if(objects[object_id].transparent) {
                double n1 = 1;
                double n2 = 1.4;
                Vector Nt = N;
                double t_n_coeff;
                Vector t_Tangent;

                if (dot(r.u, N)>0) {
                    std::swap(n1,n2);
                    Nt = -1* Nt;
                }

                t_Tangent = n1 / n2 * (r.u - dot(r.u, Nt) * Nt);
                //t_n_coeff = 1 - sqrt(1 - sqr(n1/n2) * ( 1- sqr(dot(r.u, Nt))));
                double rad = 1 - sqr(n1/n2)* (1-sqr(dot(r.u, Nt)));

                if (rad < 0) {
                    Vector ref_dir = r.u - 2*dot(r.u, N)*N;
                    ref_dir.normalize();
                    Ray mirrorR = Ray(P + 0.001*N, ref_dir);
                    return getColor(mirrorR, bounce_number-1);
                }

                t_n_coeff = -sqrt(rad);
                Vector ray_dir = t_n_coeff * Nt + t_Tangent;
                ray_dir.normalize();
                Ray refraction(P-0.001*Nt, ray_dir);
                return getColor(refraction, bounce_number -1);
            }

            Vector lightDir = L - P;
            double d2 = lightDir.norm2();
            lightDir.normalize();

            Vector shadowP, shadowN;
            int shadow_id;
            double shadow_t;

            Ray shadow_ray(P + 0.001*N, lightDir);
            bool shadow = false;
            if(intersect(shadow_ray, shadowP, shadowN, shadow_t, shadow_id)){
                
                if((shadowP - P).norm2() < d2){
                    shadow = true;
                }
            }
            if (!shadow) {
            Vector color = I/(4 * M_PI * d2) * objects[object_id].alb / M_PI * std::max(0., dot(lightDir, N));
            return color;
            }
    
            return Vector(0,0,0);
        }

        /* Vector getColor(const Ray& r, int bounce) {
            if (bounce <= 0) return Vector(0, 0, 0);
    
            Vector P, N;
            double t;
            int id;
            if (!intersect(r, P, N, t, id)) return Vector(0, 0, 0);
    
            Object* obj = objects[id];
    
            if (obj->mirror) {
                Vector reflected_dir = r.u - 2*dot(r.u, N)*N;
                Ray reflected_ray(P + 0.001*N, reflected_dir);
                return getColor(reflected_ray, bounce - 1);
            }
    
            if (obj->transparent) {
                double n1 = 1, n2 = 1.4;
                Vector N_transp = N;
                if (dot(r.u, N) > 0) {
                    std::swap(n1, n2);
                    N_transp = -1*N_transp;
                }
                Vector T_tangent = (n1/n2)*(r.u - dot(r.u, N_transp)*N_transp);
                double rad = 1 - sqr(n1/n2)*(1-sqr(dot(r.u, N_transp)));
                if (rad < 0) {
                    Vector reflected_dir = r.u - 2*dot(r.u, N)*N;
                    Ray reflected_ray(P - 0.001*N, reflected_dir);
                    return getColor(reflected_ray, bounce-1);
                }
                Vector T_normal = -sqrt(rad)*N_transp;
                Ray refracted_ray(P - N_transp*0.001, T_tangent+T_normal);
                return getColor(refracted_ray, bounce-1);
            }

        Vector light_dir = L - P;
        double d2 = light_dir.norm2();
        light_dir.normalize();

        // Shadow
        Ray shadow_ray(P + 0.001*N, light_dir);
        Vector shadowP, shadowN;
        double shadowt;
        int shadow_id;
        bool in_shadow = intersect(shadow_ray, shadowP, shadowN, shadowt, shadow_id) && (shadowP - P).norm2() < d2;

        if (!in_shadow) {
            return (I / (4 * M_PI * d2)) * (obj->alb / M_PI * std::max(dot(N, light_dir), 0.0));
        }
        else {
            return Vector(0, 0, 0);
        }
    }*/
};
// stuff lecturer did:
// put get color as a function of the scene
// shadows?
// fix which intersection comes first: your main ball disappears
/*
class BoundingBox { // this is supposed to speed it up
    public:
    BoundingBox(const Vector& m = Vector(0,0,0), const Vector & M = Vector(0,0,0)) : m(m), M(M) {};

    bool intersect(const Ray& r) { // checks if theres an intersection with a ray and the bounding box
        // check this in some intersect function

        double tx1 = (m[0] - r.O[0]/r.u[0]);
        double tx2 = (M[0] - r.O[0]/r.u[0]);
        double txMin = std::min(tx1, tx2);
        double txMax = std::max(tx1, tx2);

        double ty1 = (m[1] - r.O[1]/r.u[1]);
        double ty2 = (M[1] - r.O[1]/r.u[1]);
        double tyMin = std::min(ty1, ty2);
        double tyMax = std::max(ty1, ty2);

        double tz1 = (m[2] - r.O[2]/r.u[2]);
        double tz2 = (M[2] - r.O[2]/r.u[2]);
        double tzMin = std::min(tz1, tz2);
        double tzMax = std::max(tz1, tz2);

        //intersect if
        if (std::min(txMax, std::min(tyMax, tzMax)) > std::max(txMin, std::max(tyMin, tzMin))) {
            return true;
        }
        return false;


    }
    Vector m, M; //min and max
};


class TriangleIndices {
    public:
        TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group) {
        };
        int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
        int uvi, uvj, uvk;  // indices within the uv coordinates array
        int ni, nj, nk;  // indices within the normals array
        int group;       // face group, gives the group of the texture
};
    

class TriangleMesh {
    public:
      ~TriangleMesh() {}
        TriangleMesh() {};
        void compute_bbox() {
            bounding_box.m = Vector(1E9, 1E9, 1E9); // progressively try to improve these bounds (min and max)
            bounding_box.m = Vector(-1E9,-1E9, -1E9);

            for (int i = 0; i < vertices.size(); i++) {

                for (int j = 0; j < 3; j++) {
                bounding_box.m[j] = std::min(bounding_box.m[j], vertices[i][j]);
                bounding_box.M[j] = std::max(bounding_box.m[j], vertices[i][j]);


                }
            }
        }
    
        std::vector<TriangleIndices> indices;
        std::vector<Vector> vertices;
        std::vector<Vector> normals;
        std::vector<Vector> uvs;
        std::vector<Vector> vertexcolors;
        //BoundingBox bounding_box;
    };

    */
int main() {
    int W = 512;
    int H = 512;

    Vector camera_origin(0, 0, 55); //camera placement
    double fov = 60 * M_PI / 180; //field of view
            
    Scene scene;
    scene.L = Vector(-10,20,40);
    scene.I = 2E10;
    Sphere s(Vector(0,0,-10), 2, Vector(1,0,0), false, false); // Big red ball
    scene.add(s);

    //TriangleMesh mesh;
    //scene.add(&mesh);

    scene.add(Sphere(Vector(0,-1000,0), 990, Vector(1, 0, 0), false, false)); // bottom
    scene.add(Sphere(Vector(0,1000,0), 980, Vector(0.2, 0.5, 0.9), false, false)); // top
    scene.add(Sphere(Vector(0,0,1000), 980, Vector(0.4, 0.8, 0.7), false, false)); // back, covers all other balls or doesnt appear at all
    scene.add(Sphere(Vector(1000,0,0), 980, Vector(0.9, 0.2, 0.9), false, false)); //left and right
    scene.add(Sphere(Vector(-1000,0,0), 980, Vector(0.6, 0.5, 0.1), false, false)); // left and right


    std::vector<unsigned char> image(W * H * 3, 0);
    int bounce_number = 5;

    // iterating through every pixel (i,j)
    for (int i = 0; i < H; i++) {
        for (int j = 0; j < W; j++) {

            double d = -W/( 2. * tan(fov / 2.) );

            Vector r_dir(j - W/2 + 0.5, -i + 0.5 + H/2, d); // ray directions
            r_dir.normalize(); // normalize ray
            Ray r(camera_origin, r_dir); // ray for this pixel specifically
            Vector color = scene.getColor(r, bounce_number);

            
            //image[(i * W + j) * 3 + 0] =(std::max(0., std::min(255., std::pow(color[0], 1/2.2))));
            //image[(i * W + j) * 3 + 1] = (std::max(0., std::min(255., std::pow(color[1], 1/2.2))));
            //image[(i * W + j) * 3 + 2] =(std::max(0., std::min(255., std::pow(color[2], 1/2.2))));

            //image[(i * W + j) * 3 + 0] = std::pow(color[0], 1/2.2);
            //image[(i * W + j) * 3 + 1] = std::pow(color[1], 1/2.2);
            //image[(i * W + j) * 3 + 2] = std::pow(color[2], 1/2.2);
            
            //image[(i * W + j) * 3 + 0] = to_byte(color[0]);
            //image[(i * W + j) * 3 + 1] = to_byte(color[1]);
            //image[(i * W + j) * 3 + 2] = to_byte(color[2]);
            

            /*image[(i * W + j) * 3 + 0] = std::min(255., color[0]);
            image[(i * W + j) * 3 + 1] = std::min(255., color[1]);
            image[(i * W + j) * 3 + 2] = std::min(255., color[2]);
                */
            /*
            image[3 * (i * W + j) + 0] = 255 * std::pow(color[0], 1/2.2);
            image[3 * (i * W + j) + 1] = 255 * std::pow(color[1], 1/2.2);
            image[3 * (i * W + j) + 2] = 255 * std::pow(color[2], 1/2.2);
           */
            image[(i * W + j) * 3 + 0] = std::min(255., pow(color[0], 0.44));
            image[(i * W + j) * 3 + 1] = std::min(255., pow(color[1], 0.44));
            image[(i * W + j) * 3 + 2] = std::min(255., pow(color[2], 0.44));
           /*Vector P, N;
            double t;
            if (S.intersect(r, P, N, t)) {
                image[(i * W + j) * 3 + 0] = (255.);
                image[(i * W + j) * 3 + 1] = (255.);
                image[(i * W + j) * 3 + 2] = (255.); }
            }*/


        
        }
            
        }
    stbi_write_png("lab_image.png", W, H, 3, &image[0], 0);
 
    return 0;
}
/*
object class
triangle mesh class inherits from object class, has intersect function



*/