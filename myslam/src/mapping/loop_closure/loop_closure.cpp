#include "myslam/mapping/loop_closure/loop_closure.hpp"

#include <cmath>
#include <algorithm>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "myslam/models/registration/ndt_registration.hpp"
#include "myslam/models/cloud_filter/voxel_filter.hpp"
#include "myslam/tools/file_manager.hpp"

const std::string WORK_SPACE_PATH = "/home/yihe/catkin_ws/src/Slam/myslam";

namespace myslam {
LoopClosing::LoopClosing() {
    InitwithConfig();
}

bool LoopClosing::InitwithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/loop_closing/config.yaml";
    LOG(INFO) << "Loading loop closure configuration file: " << config_file_path << std::endl;
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::cout << "--------Loop Closure Parameters Initialization---------" << std::endl;
    InitParam(config_node);
    InitDataPath(config_node);
    InitRegistration(registration_ptr_, config_node);
    InitFilter("map", map_filter_ptr_, config_node);
    InitFilter("scan", scan_filter_ptr_, config_node);

    return true;
}

bool LoopClosing::InitParam(const YAML::Node &config_node) {
    extend_frame_num_ = config_node["extend_frame_num"].as<int>();
    loop_step_ = config_node["loop_step"].as<int>();
    diff_num_ = config_node["diff_num"].as<int>();
    detect_area_ = config_node["detect_area"].as<float>();
    fitness_score_limit_ = config_node["fitness_score_limit"].as<float>();

    return true;
}

bool LoopClosing::InitDataPath(const YAML::Node &config_node) {
    std::string data_path = config_node["data_path"].as<std::string>();
    if (!FileManager::CreateDirectory(data_path)) {
        return false;
    }

    key_frames_path_ = data_path + "/key_frames";

    return true;
}

bool LoopClosing::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node &config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    LOG(INFO) << "Point registration method is " << registration_method;

    if (registration_method == "NDT") {
        registration_ptr_ = std::make_shared<NDTRegistration>(config_node["NDT"]);
    } else {
        LOG(ERROR) << "Cannot find configurations for" << registration_method << std::endl;
        return false;
    }
}

bool LoopClosing::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node) {
    std::string filter_method  = config_node[filter_user + "_filter"].as<std::string>();
    LOG(INFO) << filter_user << " use " << filter_method << " as the filter";

    if (filter_method == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);

    } else {
        LOG(ERROR) << "Cannot find configurations for " << filter_method << std::endl;
        return false;
    }

    return true;
}


bool LoopClosing::Update(const KeyFrame keyframe, const KeyFrame key_gnss) {
    has_new_loop_pose_ = false;

    all_key_frames_.push_back(keyframe);
    all_key_gnss_.push_back(key_gnss);

    int key_frame_index = 0;
    if (!DetectNearestKeyFrame(key_frame_index)) {
        return false;
    }

    if (!CloudRegistration(key_frame_index)) {
        return false;
    }
}

bool LoopClosing::HasNewLoopPose() {
    return has_new_loop_pose_;
}

LoopPose& LoopClosing::GetCurrentLoopPose(){
    return current_loop_pose_;
}

bool LoopClosing::DetectNearestKeyFrame(int &key_frame_index) {
    static int skip_cnt = 0;
    static int skip_num = loop_step_;

    std::cout << "Detecting nearest key frame: " << std::endl;
    std::cout << "Skip count: " << skip_cnt << std::endl;
    std::cout << "Size of all key frames: " << all_key_frames_.size() << std::endl;
    std::cout << "Size of all gnss: " << all_key_gnss_.size() << std::endl;

    if (++skip_cnt < skip_num) {
        return false;
    }

    if ((int)all_key_gnss_.size() < diff_num_ + 1) {
        return false;
    }

    int key_num = (int)all_key_gnss_.size();
    float min_distance = 1e6;
    float distance = 0.0;

    KeyFrame history_key_frame;
    KeyFrame current_key_frame = all_key_gnss_.back();

    key_frame_index = -1;
    for (int i=0; i<key_num-1; ++i) {
        if (key_num - i < diff_num_) break;

        history_key_frame = all_key_gnss_.at(i);
        distance = fabs(current_key_frame.pose(0,3) - history_key_frame.pose(0,3)) + 
                   fabs(current_key_frame.pose(1,3) - history_key_frame.pose(1,3)) +
                   fabs(current_key_frame.pose(2,3) - history_key_frame.pose(2,3));

        if (distance < min_distance) {
            min_distance = distance;
            key_frame_index = i;
        }
    }

    if (key_frame_index < extend_frame_num_) {
        return false;
    }

    skip_cnt = 0;
    skip_num = (int)min_distance;
    if (min_distance > detect_area_) {
        skip_num = std::max((int)(min_distance/2.0), loop_step_);
        return false;
    } else {
        skip_num = loop_step_;
        return true;
    }
}

bool LoopClosing::CloudRegistration(int key_frame_index) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Matrix4f map_pose = Eigen::Matrix4f::Identity();
    JointMap(key_frame_index, map_cloud_ptr, map_pose);

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Matrix4f scan_pose = Eigen::Matrix4f::Identity();
    JointScan(scan_cloud_ptr, scan_pose);

    Eigen::Matrix4f result_pose = Eigen::Matrix4f::Identity();
    Registration(map_cloud_ptr, scan_cloud_ptr, scan_pose, result_pose);

    current_loop_pose_.pose = map_pose.inverse() * result_pose;

    if (registration_ptr_->GetFitnessScore() > fitness_score_limit_) {
        return false;
    }

    static int loop_close_cnt = 0;
    loop_close_cnt++;

    std::cout << "Loop closure detected: " << loop_close_cnt << ": Frame " << current_loop_pose_.index0 << "-------> Frame" << current_loop_pose_.index1 << std::endl;
    std::cout << "Fitness score: " << registration_ptr_->GetFitnessScore() << std::endl;

    return true;
}

bool LoopClosing::JointMap(int key_frame_index, pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_ptr, Eigen::Matrix4f &map_pose) {
    map_pose = all_key_gnss_.at(key_frame_index).pose;
    current_loop_pose_.index0 = all_key_frames_.at(key_frame_index).index;

    Eigen::Matrix4f pose_to_gnss = map_pose * all_key_frames_.at(key_frame_index).pose.inverse();

    for (int i = key_frame_index - extend_frame_num_; i<key_frame_index+extend_frame_num_; ++i) {
        std::string file_path = key_frames_path_ + "/key_frame_" + std::to_string(all_key_frames_.at(i).index) + ".pcd";

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::io::loadPCDFile(file_path, *cloud_ptr);

        Eigen::Matrix4f cloud_pose = pose_to_gnss  * all_key_frames_.at(i).pose;
        pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, cloud_pose);

        *map_cloud_ptr += *cloud_ptr;
    }
    map_filter_ptr_->Filter(map_cloud_ptr, map_cloud_ptr);
    return true;
}

bool LoopClosing::JointScan(pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud_ptr, Eigen::Matrix4f &scan_pose) {
    scan_pose = all_key_gnss_.back().pose;
    current_loop_pose_.index1 = all_key_frames_.back().index;
    current_loop_pose_.time = all_key_frames_.back().time;

    std::string file_path = key_frames_path_ + "key_frame_" + std::to_string(all_key_frames_.back().index) + ".pcd";
    pcl::io::loadPCDFile(file_path, *scan_cloud_ptr);
    scan_filter_ptr_->Filter(scan_cloud_ptr, scan_cloud_ptr);

    return true;
}

bool LoopClosing::Registration(pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_ptr,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud_ptr,
                    Eigen::Matrix4f& scan_pose,
                    Eigen::Matrix4f& result_pose) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    registration_ptr_->setInputTarget(map_cloud_ptr);
    registration_ptr_->PointCloudAlign(scan_cloud_ptr, scan_pose, result_cloud_ptr, result_pose);

    return true;
}

}// namespace myslam