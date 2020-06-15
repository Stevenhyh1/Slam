#ifndef MYSLAM_MODELS_DISTORTION_ADJUST_DISTORTION_ADJUST_HPP_
#define MYSLAM_MODELS_DISTORTION_ADJUST_DISTORTION_ADJUST_HPP_

#include <pcl/common/transforms.h>
#include <Eigen/Dense>

#include "myslam/sensor_data/cloud_data.hpp"
#include "myslam/sensor_data/velocity_data.hpp"

namespace myslam {
class DistortionAdjust
{
public:
    DistortionAdjust() = default;
    void SetMotionInfo(float scan_period, VelocityData velocity_data);
    bool AdjustCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud_ptr);

private:
    float scan_period_;
    Eigen::Vector3f velocity_;
    Eigen::Vector3f angular_rate_;

    Eigen::Matrix3f UpdataMatrix(float real_time);

};
} //namespace myslam
#endif