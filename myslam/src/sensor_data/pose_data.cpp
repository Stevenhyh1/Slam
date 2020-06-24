#include "myslam/sensor_data/pose_data.hpp"

namespace myslam {

Eigen::Quaternionf PoseData::GetQuaternion() {
    Eigen::Matrix3f rotation_matrix = pose.block<3,3>(0,0);
    Eigen::Quaternionf q(rotation_matrix);

    return q;
}

} //namespace myslam