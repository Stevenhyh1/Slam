#include "myslam/mapping/front_end/front_end.hpp"

#include <cmath>
#include <iostream>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include "glog/logging.h"

// #include "myslam/global_definition.hpp"
const std::string WORK_SPACE_PATH = "/home/yihe/catkin_ws/src/Slam/myslam";

namespace myslam{
FrontEnd::FrontEnd():local_map_ptr_(new pcl::PointCloud<pcl::PointXYZ>){
    // Default parameters
    InitWithConfig();
    }

bool FrontEnd::InitWithConfig() {
    // std::string config_file_path = "/home/yihe/catkin_ws/src/Slam/myslam/config/front_end/config.yaml";
    std::string config_file_path = WORK_SPACE_PATH + "/config/front_end/config.yaml";
    LOG(INFO) << "Loading front end configuration file: " << config_file_path << std::endl;
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    // LOG(INFO) << "Config Loaded!" << std::endl;

    InitParam(config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    InitFilter("frame", current_frame_filter_ptr_, config_node);
    InitRegistration(registration_ptr_, config_node);

    return true;
}

bool FrontEnd::InitParam(const YAML::Node &node) {
    local_frame_num = node["local_frame_num"].as<int>();
    key_frame_distance = node["key_frame_distance"].as<float>();

    LOG(INFO) << "Number of key frames for local map is: " << local_frame_num << std::endl;
    LOG(INFO) << "Minimum distances between two key frames are: " << key_frame_distance << std::endl;

    return true;
}

bool FrontEnd::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &node) {
    std::string filter_method  = node[filter_user + "_filter"].as<std::string>();
    LOG(INFO) << filter_user << "use " << filter_method << " as the filter";

    if (filter_method == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(node[filter_method][filter_user]);

    } else {
        LOG(ERROR) << "Cannot find configurations for " << filter_method << std::endl;
        return false;
    }

    return true;
}

bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface> &registration_ptr, const YAML::Node &node) {
    std::string registration_method = node["registration_method"].as<std::string>();
    LOG(INFO) << "Point registration method is " << registration_method;
    
    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(node[registration_method]);
    } else {
        LOG(ERROR) << "Cannot find configurations for" << registration_method << std::endl;
        return false;
    }

    return true;
}

bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;
    return true;
}

bool FrontEnd::Update(const CloudData& cloud_data, Eigen::Matrix4f &cloud_pose) {

    current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices); // out[i] = in[indices[i]]

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    current_frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr, filtered_cloud_ptr);

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    // If the local map contatiner is empty, this point cloud is the first one
    // Insert current point cloud as a key frame, and update maps
    if (local_map_frames_.empty()) {
        current_frame_.pose = init_pose_;
        UpdateNewFrame(current_frame_);
        cloud_pose = current_frame_.pose;
        return true;
    }

    // NDT matching
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZ>);
    registration_ptr_->PointCloudAlign(filtered_cloud_ptr, predict_pose, result_cloud_ptr_, current_frame_.pose);
    cloud_pose = current_frame_.pose;

    step_pose = last_pose.inverse()*current_frame_.pose;
    predict_pose = current_frame_.pose*step_pose;
    last_pose = current_frame_.pose;

    if (fabs(last_key_frame_pose(0,3)-current_frame_.pose(0,3)) + 
        fabs(last_key_frame_pose(1,3)-current_frame_.pose(1,3)) + 
        fabs(last_key_frame_pose(2,3)-current_frame_.pose(2,3)) > 2.0) {
            UpdateNewFrame(current_frame_);
            last_key_frame_pose = current_frame_.pose;
        }

    return true;
}

void FrontEnd::UpdateNewFrame(const Frame& new_key_frame) {

    Frame key_frame = new_key_frame;
    key_frame.cloud_data.cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>(*new_key_frame.cloud_data.cloud_ptr));
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    //Update local map
    local_map_frames_.push_back(key_frame);
    while (local_map_frames_.size() > static_cast<size_t>(local_frame_num)) {
        local_map_frames_.pop_front();
    }
    local_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i=0; i<local_map_frames_.size(); ++i) {
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, *transformed_cloud_ptr, local_map_frames_.at(i).pose);
        *local_map_ptr_ += *transformed_cloud_ptr;
    }

    //Update ndt matching targets
    if (local_map_frames_.size() < 10) {
        registration_ptr_->setInputTarget(local_map_ptr_);
    } else {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_local_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
        registration_ptr_->setInputTarget(filtered_local_map_ptr);
    }

    return;
}
} // namespace myslam