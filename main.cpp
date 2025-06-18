#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <vector>
#include <cmath>
#include <random>
#include <string>
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <list>

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


constexpr double PI = 3.14159265358979323846;

Vector random_vector() {
	double rand1 = uniform(engine);
	double rand2 = uniform(engine);
	double x1 = std::cos(2*PI*rand1) * std::sqrt(rand2*(1-rand2));
	double x2 = std::sin(2*PI*rand1) * std::sqrt(rand2*(1-rand2));
	double x3 = 1 - 2*rand2;
	return Vector(x1, x2, x3);
}

using Image = std::vector<Vector>;

void sliced_optimal_transport(Image& I, const Image& M) {
    int n = I.size();
	int N = 100;

    for (int i = 0; i < N; i++) {
        Vector v = random_vector();

        std::vector<std::pair<double, int>> projI(n), projM(n);

        for (int i = 0; i < n; ++i) {
            projI[i] = {dot(I[i], v), i};
            projM[i] = {dot(M[i], v), i};
        }

        std::sort(projI.begin(), projI.end());
        std::sort(projM.begin(), projM.end());

        for (int i = 0; i < n; ++i) {
            int index = projI[i].second;
            double t = projM[i].first - projI[i].first;
            for (int j = 0; j < 3; ++j) {
                I[index][j] += t * v[j];
            }
        }
    }
}



int main() {

	int W, H, C;
	
	//stbi_set_flip_vertically_on_load(true);
	unsigned char *image = stbi_load("8733654151_b9422bb2ec_k.jpg",
                                 &W,
                                 &H,
                                 &C,
                                 STBI_rgb);
	std::vector<double> image_double(W*H*3);
	for (int i=0; i<W*H*3; i++)
		image_double[i] = image[i];
	
	std::vector<unsigned char> image_result(W*H*3, 0);

	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {

			image_result[(i*W + j) * 3 + 0] = image_double[(i*W+j)*3+0]*0.5;
			image_result[(i*W + j) * 3 + 1] = image_double[(i*W+j)*3+1]*0.3;
			image_result[(i*W + j) * 3 + 2] = image_double[(i*W+j)*3+2]*0.2;
		}
	}
	
	int N = W * H;

	Image M(N), I(N);
	for (int i = 0; i < N; ++i) {
		M[i][0] = image_double[i*3 + 0];
		M[i][1] = image_double[i*3 + 1];
		M[i][2] = image_double[i*3 + 2];

		I[i][0] = image_result[i*3 + 0];
		I[i][1] = image_result[i*3 + 1];
		I[i][2] = image_result[i*3 + 2];
	}

	sliced_optimal_transport(I, M);

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < 3; j++) {
            image_result[i*3 + j] = I[i][j];
        }
    }

	stbi_write_png("image.png", W, H, 3, &image_result[0], 0);
	return 0;
}