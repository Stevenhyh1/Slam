#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

int main(int argc, char *argv[]) {

    //Define rotation matrix, rotation vector
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    Eigen::AngleAxisd rotation_vector (M_PI/4, Eigen::Vector3d (0,0,1));
    // cout .precision(3);
    cout << "Rotation Matrix =\n" << rotation_vector.matrix() << endl;
    cout << "Rotation Matrix =\n" << rotation_vector.toRotationMatrix() << endl;
    rotation_matrix = rotation_vector.toRotationMatrix();

    //Rotation
    Eigen::Vector3d v (1,0,0);
    Eigen::Vector3d v_rotated = rotation_vector * v;
    cout << "Transform from rotation vector: \n" << v_rotated << endl;
    v_rotated = rotation_matrix * v;
    cout << "Transform form rotation matrix: \n" << v_rotated << endl;

    //Transform to Euler angles
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2,1,0);
    cout << "yaw, pitch, row: " << euler_angles << endl;

    //Transformation Matrix
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(rotation_matrix);
    T.pretranslate(Eigen::Vector3d (1,3,4));
    cout << "Transformation matrix: \n" << T.matrix() << endl;

    //Transformation
    Eigen::Vector3d v_transformed = T*v;
    cout << "V transformed: \n" << v_transformed << endl;

    //Quaternion
    Eigen::Quaterniond q = Eigen::Quaterniond (rotation_matrix);
    cout << "quaternion: \n" << q.coeffs() << endl;

    q = Eigen::Quaterniond (rotation_vector);
    cout << "quaternion from vector: \n" << q.coeffs() << endl;

    v_rotated = q*v;
    cout << "Transform from quaternion: \n" << v_rotated << endl;

    double x1 = 1.0, y1 = 1.0;
    double x2 = 1.0, y2 = -1.0;
    double x3 = -1.0, y3 = -1.0;
    double x4 = -1.0, y4 = 1.0;

    std::cout << atan2(y1, x1) << std::endl;
    std::cout << atan2(y2, x2) << std::endl;
    std::cout << atan2(y3, x3) << std::endl;
    std::cout << atan2(y4, x4) << std::endl;
}