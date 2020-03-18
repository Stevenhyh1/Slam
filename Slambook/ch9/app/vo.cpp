#include <iostream>
#include <utils.h>
#include <camera.h>

using namespace std;

int main() {
    myvo::Camera a (1,1,1,1,1);
    cout << "Create Camera" << endl;
    Eigen::Vector3d v (1,0,0);
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d (0,0,1)).toRotationMatrix();
    Eigen::Vector3d t(1,0,0);
    Sophus::SE3d s(R,t);
    Eigen::Vector3d cam_crd = a.world2camera( v, s);
    cout << cam_crd << endl;
}