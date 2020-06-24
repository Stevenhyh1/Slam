#include "myslam/sensor_data/key_frame.hpp"

namespace myslam {

Eigen::Quaternionf KeyFrame::GetQuaternion() {
    Eigen::Matrix3f rotation_matrix = pose.block<3,3>(0,0);
    Eigen::Quaternionf q(rotation_matrix);

    return q;
}

} //namespace myslam