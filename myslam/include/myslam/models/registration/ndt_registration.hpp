#ifndef MYSLAM_MODELS_REGISTRATION_NDT_REGISTRATION_HPP_
#define MYSLAM_MODELS_REGISTRATION_NDT_REGISTRATION_HPP_

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include "myslam/models/registration/registration_interface.hpp"

namespace myslam {
class NDTRegistration : public RegistrationInterface
{
private:

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_ptr_;

public:
    NDTRegistration(const YAML::Node &node);
    NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

    bool setInputTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud) override;
    bool PointCloudAlign(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_ptr, 
                         const Eigen::Matrix4f &predict_pose,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &result_ptr,
                         Eigen::Matrix4f &result_pose) override;

private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

};
}

#endif