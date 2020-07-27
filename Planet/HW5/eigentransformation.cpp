#include <iostream>
#include <cmath>

#include <Eigen/Geometry>
#include <Eigen/Core>

int main(int argc, char *argv[]) {

    // ---------Initailization---------- //
    // Rotation Vector: rotate 45 degree around z axis
    Eigen::AngleAxisd rotation_vector(M_PI_4, Eigen::Vector3d::UnitZ());
    std::cout << "rotation_vector axis = \n" << rotation_vector.axis() << "\nrotation_vector angle = " << rotation_vector.angle() << std::endl;

    // Rotation Matrix:
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    rotation_matrix << 0.707, -0.707, 0, 
                       0.707, 0.707 , 0,
                       0    , 0     , 1;
    std::cout << "rotation matrix = \n" << rotation_matrix << std::endl;

    // Quaternion
    Eigen::Quaterniond quat = Eigen::Quaterniond(0, 0, 0.383, 0.924); // (w, x, y, z) for constructor
    std::cout << "quaternion = \n" << quat.coeffs() << std::endl; // (x, y, z, w) for storing

    // Euler Angle
    Eigen::Vector3d euler_angles = Eigen::Vector3d(M_PI_4, 0, 0); //ZYX
    std::cout << "Euler: yaw pitch roll = \n" << euler_angles.transpose() << std::endl;

    // ---------Transformation---------- //
    // Rotation vector to other format
    std::cout << "Rotation vector to rotation matrix 1: rotation matrix = \n" << rotation_vector.toRotationMatrix() << std::endl;
    std::cout << "Rotation vector to rotation matrix 2: rotation matrix = \n" << Eigen::Matrix3d(rotation_vector) << std::endl;

    quat = Eigen::Quaterniond(rotation_vector);
    std::cout << "Rotation vector to quaternion = \n" << quat.coeffs() << std::endl;

    // Rotation matrix to other format
    rotation_vector.fromRotationMatrix(rotation_matrix);
    std::cout << "Rotation matrix to rotation vector: rotation_vector axis = \n" << rotation_vector.axis() 
              << "\n rotation_vector angle = " << rotation_vector.angle() << std::endl;
    
    rotation_vector = Eigen::AngleAxisd(rotation_matrix);
    std::cout << "Initialize rotation vector directly with rotation matrix: rotation_vector axis: \n" << rotation_vector.axis()
              << "\n rotation_vector angle = " << rotation_vector.angle() << std::endl;

    euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
    std::cout << "Rotation matrix to euler angles: yaw pitch roll = " << euler_angles.transpose() << std::endl;

    quat = Eigen::Quaterniond(rotation_matrix);
    std::cout << "Transformation matrix to quaternion = \n" << quat.coeffs() << std::endl;

    // Quaternion to other format
    rotation_vector = Eigen::AngleAxisd(quat);
    std::cout << "Quaternion to rotation vector: rotation_vector = \n" << rotation_vector.axis()
              << "\n rotation vector angle = " << rotation_vector.angle() << std::endl;

    rotation_matrix = Eigen::Matrix3d(quat);
    std::cout << "Quaternion to rotation matrix 1: rotation matrix = \n" << rotation_matrix << std::endl;

    rotation_matrix = quat.toRotationMatrix();
    std::cout << "Quaternion to rotation matrix 2: rotation matrix = \n" << rotation_matrix << std::endl;

    // Transformation matrix
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translate(Eigen::Vector3d(1, 2, 3));
    T.rotate(rotation_matrix);
    

    std::cout << "Transformation matrix = \n" << T.matrix() << std::endl;

    std::cout << "Rotation matrix = \n" << T.rotation() << std::endl;

    std::cout << "Translation vector = \n" << T.translation() << std::endl;

    return 0;
}