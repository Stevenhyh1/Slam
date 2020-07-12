#include "myslam/models/registration/ndt_registration.hpp"

namespace myslam {

NDTRegistration::NDTRegistration(const YAML::Node &node)
:ndt_ptr_(new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>()) {
    float res = node["res"].as<float>();
    float step_size = node["step_size"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iter = node["max_iter"].as<float>();

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
    
}

NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps, int max_iter) {
    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

float NDTRegistration::GetFitnessScore() {
    return ndt_ptr_->getFitnessScore();
}

bool NDTRegistration::setInputTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud) {
    ndt_ptr_->setInputTarget(target_cloud);
    return true;
}

bool NDTRegistration::PointCloudAlign(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_ptr,
                        const Eigen::Matrix4f &predict_pose,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &result_ptr,
                        Eigen::Matrix4f &result_pose) {
                            // double start = ros::Time::now().toSec();
                            ndt_ptr_->setInputSource(source_ptr);
                            ndt_ptr_->align(*result_ptr, predict_pose);
                            result_pose = ndt_ptr_->getFinalTransformation();
                            // double end = ros::Time::now().toSec();
                            // std::cout << "NDT matching time: " << end - start << std::endl;
                            return true;
                        }

bool NDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter) {
    ndt_ptr_->setResolution(res);
    ndt_ptr_->setStepSize(step_size);
    ndt_ptr_->setTransformationEpsilon(trans_eps);
    ndt_ptr_->setMaximumIterations(max_iter);
    return true;
}
} //namespace myslam