#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
 #include <cmath>
  #include <iostream>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
 
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"




double sqr(double x){

    return  std::pow(x, 2);
}

double max(double x, double y){

    if(x > y){
        return x;
    }
    return y;
}
 
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
Ray(const Vector& O, const Vector& u) : O(O), u(u) {};
    Vector O;
    Vector u;
};
 

class Sphere {
public:
Sphere(const Vector& C, double R, const Vector& rho, bool is_mirror, bool is_transparent) : C(C), R(R), rho(rho), is_mirror(is_mirror), is_transparent(is_transparent) {};
    Vector C;
    Vector P;
    double R;
    Vector rho;
    bool is_mirror = false;
    bool is_transparent = false;

    bool intersect(const Ray& r, Vector& P,  Vector &N, double &t){

        double delta = sqr(dot(r.u, r.O - C)) - ((r.O - C).norm2() - R*R);

        if(delta >= 0){
            
            double t1 = dot(r.u, C - r.O) - sqrt(delta);
            double t2 = dot(r.u, C - r.O) + sqrt(delta);
            if(t2 < 0) {return false;}
            if(t1 > 0) {
                t = t1;
            }
            else{
                t = t2;
            }
            P = r.O + t*r.u;
            N = P - C;
            N.normalize();
            return true;

        }

        return (delta >= 0);
    }
};
 


class Scene{

    public:
    double I;
    Vector L;
    
    std::vector<Sphere> objects;

    void addSphere(const Sphere& s){objects.push_back(s);}

    bool intersect(const Ray& r, Vector& P,  Vector &N, double &t, int& sphere_id){

            bool has_inter= false;
            t = std::numeric_limits<double>::max();
            Vector localP, localN;
            double localt;

            for (int i = 0; i< objects.size(); i++){
                

                if(objects[i].intersect(r, localP, localN, localt)){


                    if(localt < t){

                        P = localP;
                        N = localN;
                        t = localt;
                        has_inter = true;
                        sphere_id = i;

                    }
                }


            }
            return has_inter;

    }

    Vector getColor(const Ray r, int bounce){

        //else diffuse sphere:
        Vector P, N;
        double t;
        Vector color(0, 0, 0);
        int sphere_id;

        if(bounce <=0){
            return Vector(0, 0, 0);
        }

        if(intersect(r, P, N, t, sphere_id)){
            if(objects[sphere_id].is_mirror){
                                    

                Vector mirrorDirection = r.u - 2*dot(r.u, N)*N;
                Ray mirrorR(P + 0.001*N, mirrorDirection);
                return getColor(mirrorR, bounce - 1);

            }

            if (objects[sphere_id].is_transparent) {
				double n1 = 1;
				double n2 = 1.4;
				Vector Ntransp = N;
				if (dot(r.u, N)>0) {
					std::swap(n1, n2);
					Ntransp = -1* Ntransp;
				}

				Vector tTangent, tNormal;
				tTangent = n1/n2 * (r.u - dot(r.u, Ntransp) * Ntransp);
				double rad = 1 - sqr(n1/n2)* (1-sqr(dot(r.u, Ntransp)));

                if (rad<0) {
					Vector mirrorDirection = r.u - 2*dot(r.u, N)*N;
					Ray mirrorR(P - 0.001*N, mirrorDirection);
					return getColor(mirrorR, bounce-1);
                    std::cout << "bad" << std::endl;
				}
                

				tNormal = -sqrt(rad)*Ntransp;
				Ray refractedRay(P - Ntransp*0.001, tTangent+tNormal);
				return getColor(refractedRay, bounce-1);
			}

            double d2 = (L-P).norm2();
            Vector lightdir = (L - P); lightdir.normalize();

            Ray shadowRay(P + 0.001*N, lightdir);
            Vector shadowP, shadowN;
            int shadow_id;
            double shadowt;

            bool in_shadow = false;
            if(intersect(shadowRay, shadowP, shadowN, shadowt, shadow_id)){
                
                if((shadowP - P).norm2() < d2){
                    in_shadow = true;
                }
            }

            if(!in_shadow){
                color = (I/(4. * 3.14159265358979323846264338327950288 * d2)) * (objects[sphere_id].rho / 3.14159265358979323846264338327950288 * max(dot(N, lightdir), 0));
                
            }

            }

        return color;
    }




 };
int main() {
    int W = 512;
    int H = 512;
    
    Scene scene;
    Sphere S(Vector(0, 0, 0), 10, Vector(1, 1, 1), true, false);
    Sphere leftwall(Vector(-1000, 0, 0), 940, Vector(0.5, 0.8, 0.1), false, false);
    Sphere rightwall(Vector(1000, 0, 0), 940, Vector(0.9, 0.2, 0.3), false, false);
    Sphere topwall(Vector(0, 1000, 0), 940, Vector(0.3, 0.5, 0.3), false, false);
    Sphere bottomwall(Vector(0, -1000, 0), 990, Vector(0.6, 0.5, 0.7), false, false);
    Sphere wallfront(Vector(0, 0, -1000), 940, Vector(0.1, 0.6, 0.17), false, false);
    Sphere wallbehind(Vector(0, 0, 1000), 940, Vector(0.8, 0.2, 0.9), false, false);

    

    scene.addSphere(S);
    scene.addSphere(bottomwall);

    scene.addSphere(leftwall);
    scene.addSphere(rightwall);
    scene.addSphere(topwall);
    scene.addSphere(wallfront);
    scene.addSphere(wallbehind);

    int max_bounces = 6;
    Vector camera_center(0, 0, 55);
    double alpha = 3.14159265358979323846264338327950288/3;  

    scene.I = 2E10;
    Vector hello(-10, 20, 40);
    scene.L = hello;


    std::vector<unsigned char> image(W * H * 3, 0);

    for (int i = 0; i < H; i++) {
        for (int j = 0; j < W; j++) {

            Vector ray_dir;
            ray_dir[0] = j - W / 2. + 0.5;
            ray_dir[1] = -i + H / 2. - 0.5;
            ray_dir[2] = -W / (2 * tan(alpha / 2.));
            ray_dir.normalize();
            Ray r(camera_center, ray_dir);
            Vector color = scene.getColor(r, max_bounces);

            // std::cout << color[0] << " ";
            // std::cout << color[1] << " ";
            // std::cout << color[2] << std::endl;
                            
            image[(i * W + j) * 3 + 0] = std::min(255., pow(color[0], 0.44));
            image[(i * W + j) * 3 + 1] = std::min(255., pow(color[1], 0.44));
            image[(i * W + j) * 3 + 2] = std::min(255., pow(color[2], 0.44));

        }
    }

    
    stbi_write_png("initial_image.png", W, H, 3, &image[0], 0);
 
    return 0;
}

