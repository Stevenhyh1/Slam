#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "myslam/mapping/viewer/viewer.hpp"
#include "myslam/tools/file_manager.hpp"

namespace myslam
{

Viewer::Viewer() {
    InitWithConfig();
}

bool Viewer::InitWithConfig() {
    std::string config_file_path = "/home/yihe/catkin_ws/src/Slam/myslam/config/viewer/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    LOG(INFO) << "Loading viewer configuration from: " << config_file_path << std::endl;

    InitParam(config_node);
    InitDataPath(config_node);
    InitFilter("frame", frame_filter_ptr_, config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    InitFilter("global_map", global_map_filter_ptr_, config_node);

    return true;
}

bool Viewer::InitParam(const YAML::Node &config_node) {
    LOG(INFO) << "Local frame number is: " << std::endl;
    local_frame_num_ = config_node["local_frame_num"].as<int>();
    return true;
}

bool Viewer::InitDataPath(const YAML::Node &config_node) {
    data_path_ = config_node["data_path"].as<std::string>();
    key_frames_path_ = data_path_ + "/key_frames";
    map_path_ = data_path_ + "/map";

    if (!FileManager::CreateDirectory(data_path_)) {
        return false;
    }
    if (!FileManager::CreateDirectory(key_frames_path_)) {
        return false;
    }
    if (!FileManager::CreateDirectory(map_path_)) {
        return false;
    }

    LOG(INFO) << "Key frames are saved in: " << key_frames_path_ << std::endl;
    LOG(INFO) << "Maps are saved in: " << map_path_ << std::endl;

    return true;
}

bool Viewer::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_pointer, const YAML::Node &config_node) {
    std::string filter_method = config_node[filter_user + "_filter"].as<std::string>();

    if (filter_method == "voxel_filter") {
        filter_pointer = std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
        LOG(INFO) << "Filtering method for " << filter_user << " is: " << filter_method << std::endl;
    } else {
        return false;
    }

    return true;
}

bool Viewer::UpdateWithOptimizedKeyFrames(std::deque<KeyFrame>& optimized_key_frames) {
    has_new_global_map_ = false;
    
    if (optimized_key_frames.size() > 0) {
        optimized_key_frames_ = optimized_key_frames;
        optimized_key_frames.clear();
        OptimizeKeyFrames();
        has_new_global_map_ = true;
    }

    return has_new_global_map_;
}

bool Viewer::UpdateWithNewKeyFrame(std::deque<KeyFrame>& new_key_frames,
                                   PoseData transformed_data,
                                   CloudData cloud_data) {
    has_new_local_map_ = false;

    if (new_key_frames.size() > 0) {
        KeyFrame key_frame;
        for (size_t i = 0; i < new_key_frames.size(); ++i) {
            key_frame = new_key_frames.at(i);
            key_frame.pose = pose_to_optimize_ * key_frame.pose;
            all_key_frames_.push_back(key_frame);
        }
        new_key_frames.clear();
        has_new_local_map_ = true;
    }

    optimized_odom_ = transformed_data;
    optimized_odom_.pose = pose_to_optimize_ * optimized_odom_.pose;

    optimized_cloud_ = cloud_data;
    pcl::transformPointCloud(*cloud_data.cloud_ptr, *optimized_cloud_.cloud_ptr, optimized_odom_.pose);

    return true;
}

bool Viewer::OptimizeKeyFrames() {
    size_t optimized_index = 0;
    size_t all_index = 0;
    while (optimized_index < optimized_key_frames_.size() && all_index < all_key_frames_.size()) {
        if (optimized_key_frames_.at(optimized_index).index < all_key_frames_.at(all_index).index) {
            optimized_index ++;
        } else if (optimized_key_frames_.at(optimized_index).index < all_key_frames_.at(all_index).index) {
            all_index ++;
        } else {
            pose_to_optimize_ = optimized_key_frames_.at(optimized_index).pose * all_key_frames_.at(all_index).pose.inverse();
            all_key_frames_.at(all_index) = optimized_key_frames_.at(optimized_index);
            optimized_index ++;
            all_index ++;
        }
    }

    while (all_index < all_key_frames_.size()) {
        all_key_frames_.at(all_index).pose = pose_to_optimize_ * all_key_frames_.at(all_index).pose;
        all_index ++;
    }

    return true;
}

bool Viewer::JoinGlobalMap(pcl::PointCloud<pcl::PointXYZ>::Ptr &global_map_ptr) {
    JoinCloudMap(optimized_key_frames_, global_map_ptr);
    return true;
}

bool Viewer::JoinLocalMap(pcl::PointCloud<pcl::PointXYZ>::Ptr &local_map_ptr) {
    size_t begin_index = 0;
    if (all_key_frames_.size() > (size_t)local_frame_num_)
        begin_index = all_key_frames_.size() - (size_t)local_frame_num_;

    std::deque<KeyFrame> local_key_frames;
    for (size_t i = begin_index; i < all_key_frames_.size(); ++i) {
        local_key_frames.push_back(all_key_frames_.at(i));
    }

    JoinCloudMap(local_key_frames, local_map_ptr);
    return true;
}

bool Viewer::JoinCloudMap(const std::deque<KeyFrame>& key_frames, pcl::PointCloud<pcl::PointXYZ>::Ptr& map_cloud_ptr) {
    map_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    std::string file_path = "";

    for (size_t i = 0; i < key_frames.size(); ++i) {
        file_path = key_frames_path_ + "/key_frame_" + std::to_string(key_frames.at(i).index) + ".pcd";
        pcl::io::loadPCDFile(file_path, *cloud_ptr);
        pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, key_frames.at(i).pose);
        *map_cloud_ptr += *cloud_ptr;
    }
    return true;
}

bool Viewer::SaveMap() {
    if (optimized_key_frames_.size() == 0)
        return false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    JoinCloudMap(optimized_key_frames_, global_map_ptr);

    std::string map_file_path = map_path_ + "/map.pcd";
    pcl::io::savePCDFileBinary(map_file_path, *global_map_ptr);

    return true;
}

Eigen::Matrix4f& Viewer::GetCurrentPose() {
    return optimized_odom_.pose;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr& Viewer::GetCurrentScan() {
    frame_filter_ptr_->Filter(optimized_cloud_.cloud_ptr, optimized_cloud_.cloud_ptr);
    return optimized_cloud_.cloud_ptr;
}

bool Viewer::GetLocalMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& local_map_ptr) {
    JoinLocalMap(local_map_ptr);
    local_map_filter_ptr_->Filter(local_map_ptr, local_map_ptr);
    return true;
}

bool Viewer::GetGlobalMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& global_map_ptr) {
    JoinGlobalMap(global_map_ptr);
    global_map_filter_ptr_->Filter(global_map_ptr, global_map_ptr);
    return true;
}

bool Viewer::HasNewLocalMap() {
    return has_new_local_map_;
}

bool Viewer::HasNewGlobalMap() {
    return has_new_global_map_;
}
} // namespace myslam
