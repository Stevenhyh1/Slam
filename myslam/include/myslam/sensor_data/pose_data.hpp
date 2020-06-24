#ifndef MYSLAM_SENSOR_DATA_POSE_DATA_HPP_
#define MYSLAM_SENSOR_DATA_POSE_DATA_HPP_

#include "Eigen/Dense"

namespace myslam {
class PoseData
{
public:
    
    double time = 0.0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

    Eigen::Quaternionf GetQuaternion();
};
} //namespace myslam

#endif