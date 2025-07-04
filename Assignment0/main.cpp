#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int main(){

    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f,2.0f,3.0f);
    Eigen::Vector3f w(1.0f,0.0f,0.0f);
    // vector output
    std::cout << "Example of output \n";
    std::cout << v << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;
    // vector dot product
    std::cout << "Example of vector dot product \n";
    std::cout << v.dot(w) << std::endl;
    // vector cross product
    std::cout << "Example of vector cross product \n";
    std::cout << v.cross(w) << std::endl;

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i << std::endl;
    // matrix add i + j
    std::cout << "Example of matrix add \n";
    std::cout << i + j << std::endl;
    // matrix scalar multiply i * 2.0
    std::cout << "Example of matrix scalar multiply \n";
    std::cout << i * 2.0 << std::endl;
    // matrix multiply i * j
    std::cout << "Example of matrix multiply \n";
    std::cout << i * j << std::endl;
    // matrix multiply vector i * v
    std::cout << "Example of matrix multiply vector \n";
    std::cout << i * v << std::endl;

    // Assignment
    // Point P
    Eigen::Vector3f p(2.0f,1.0f,1.0f);
    // Rotate 45, translation (1,2)
    Eigen::Matrix3f m;
    float angle = 45.0f / 180.0f * acosf(-1.0f);
    m << cos(angle), -sin(angle), 1,
         sin(angle), cos(angle), 2,
         0, 0, 1;
    std::cout << m * p << std::endl;

    return 0;
}