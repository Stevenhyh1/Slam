#ifndef MYSLAM_MODELS_REGISTRATION_REGISTRATION_INTERFACE_HPP_
#define MYSLAM_MODELS_REGISTRATION_REGISTRATION_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include "Eigen/Dense"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace myslam {
class RegistrationInterface
{
private:
    /* data */
public:
    virtual ~RegistrationInterface() = default;

    virtual float GetFitnessScore() = 0;
    virtual bool setInputTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud) = 0;
    virtual bool PointCloudAlign(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_ptr, 
                                 const Eigen::Matrix4f &predict_pose,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr &result_ptr,
                                 Eigen::Matrix4f &result_pose) = 0;
        
};
} // namespace myslam

#endif