#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char *argv[]) { 
    
    Eigen::Quaterniond quat(0.1,0.35,0.2,0.3); // [w, x, y, z]
    quat.normalize(); // q /= sqrt(w*w + x*x + y*y + z*z)

    // Rotation Matrix
    Eigen::Matrix3d r = quat.toRotationMatrix();
    std::cout << "r = \n" << r << std::endl;

    // Transpose of Rotation Matrix
    Eigen::Matrix3d rt = r.transpose();
    std::cout << "rt = \n" << rt << std::endl;

    // Inverse of Rotation Matrix
    Eigen::Matrix3d rinv = r.inverse();
    std::cout << "r.inv = \n" << rinv << std::endl;

    // Matrix Multiplication
    Eigen::Matrix3d rrt = r * rt;
    std::cout << "r*rt = \n" << rrt << std::endl;


}