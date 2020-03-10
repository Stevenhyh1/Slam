#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"

using namespace std;

int main (int argc, char* argv[]) {
    // Construct SO3, rotation matrix
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d (0,0,1)).toRotationMatrix();
    Eigen::Quaterniond q(R);
    Sophus::SO3d SO3_R(R);
    Sophus::SO3d SO3_q(q);

    cout .precision(3);
    cout << "SO3 from matrix: \n" << SO3_R.matrix() << endl;
    cout << "SO3 from quaternion: \n" << SO3_q.matrix() << endl;

    Eigen::Vector3d so3 = SO3_R.log();
    cout << "so3 = \n" << so3 << endl;
    cout << "so3 hat = \n" << Sophus::SO3d::hat(so3) << endl;
    cout << "so3 hat vee = \n" << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)) << endl;
    
    //disturbance
    Eigen::Vector3d update_so3 (1e-4,0,0);
    Sophus::SO3d SO3_update = Sophus::SO3d::exp(update_so3) * SO3_R;
    cout << "SO3 updated = \n" << SO3_update.matrix() << endl;

    //SE3, transformation matrix
    Eigen::Vector3d t(1,0,0);
    Sophus::SE3d SE3_R(R,t);
    Sophus::SE3d SE3_q(q,t);
    cout << "SE3 from R,t = \n" << SE3_R.matrix() << endl;
    cout << "SE3 from q,t = \n" << SE3_q.matrix() << endl;

    Eigen::Matrix<double, 6, 1> skew = SE3_R.log();
    cout << "se3 = \n" << skew << endl;
    cout << "se3 hat = \n" << Sophus::SE3d::hat(skew) << endl;
    cout << "se3 hat vee = \n" << Sophus::SE3d::vee(Sophus::SE3d::hat(skew)) << endl;

    Eigen::Matrix<double, 6, 1> update_se3;
    update_se3.setZero();
    update_se3(0,0) = 1e-4;
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_R;
    cout << "SE3 updated = \n" << SE3_updated.matrix() << endl;

    return 0;
}
