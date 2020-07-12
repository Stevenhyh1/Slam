#include "myslam/sensor_data/loop_pose.hpp"

namespace myslam {
Eigen::Quaternionf LoopPose::GetQuaternion() {
    Eigen::Quaternionf q(pose.block<3,3>(0,0));
    return q;
}
}//namespace myslam

