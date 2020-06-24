#ifndef MYSLAM_SENSOR_DATA_KEY_FRAME_HPP_
#define MYSLAM_SENSOR_DATA_KEY_FRAME_HPP_

#include "Eigen/Dense"

namespace myslam {
class KeyFrame {
public:

    double time = 0.0;
    unsigned int index = 0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

    Eigen::Quaternionf GetQuaternion();
};
} //namespace myslam

#endif