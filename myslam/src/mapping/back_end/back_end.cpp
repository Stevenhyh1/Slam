#include "myslam/mapping/back_end/back_end.hpp"

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>

#include "myslam/tools/file_manager.hpp"

namespace myslam {
BackEnd::BackEnd() {
    InitWithConfig();
}


bool BackEnd::InitWithConfig() {
    std::string config_file_path = "/home/yihe/catkin_ws/src/Slam/myslam/config/back_end/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    bool param_inited = InitParam(config_node);
    bool optimizer_inited_ = InitGraphOptimizer(config_node);
    bool path_inited = InitDataPath(config_node);

    return param_inited && optimizer_inited_ && param_inited;
}

bool BackEnd::InitParam(const YAML::Node &config_node) {
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    return true;
}

bool BackEnd::InitGraphOptimizer(const YAML::Node& config_node) {
    std::string optimizer_type = config_node["graph_optimizer"].as<std::string>();
    if (optimizer_type == "g2o") {
        graph_optimizer_ptr_ = std::make_shared<G2OGraphOptimizer>();
    } else {
        std::cout << "Cannot find g2o" << std::endl;
        return false;
    }
    graph_optimizer_config_.use_gnss = config_node["g2o"]["use_gnss"].as<bool>();
    graph_optimizer_config_.use_loop_close = config_node["g2o"]["use_loop_close"].as<bool>();

    graph_optimizer_config_.optimize_step_with_key_frame = config_node["g2o"]["optimize_step_with_key_frame"].as<int>();
    graph_optimizer_config_.optimize_step_with_gnss = config_node["g2o"]["optimize_step_with_gnss"].as<int>();
    graph_optimizer_config_.optimize_step_with_loop = config_node["g2o"]["optimize_step_with_loop"].as<int>();

    for (int i = 0; i < 6; ++i) {
        graph_optimizer_config_.odom_edge_noise(i) =
            config_node[optimizer_type + "_param"]["odom_edge_noise"][i].as<double>();
        graph_optimizer_config_.close_loop_noise(i) =
            config_node[optimizer_type + "_param"]["close_loop_noise"][i].as<double>();
    }

    for (int i = 0; i < 3; i++) {
        graph_optimizer_config_.gnss_noise(i) =
            config_node[optimizer_type + "_param"]["gnss_noise"][i].as<double>();
    }

    return true;
}

bool BackEnd::InitDataPath(const YAML::Node &config_node) {
    std::string data_path = config_node["data_path"].as<std::string>();
    if (!FileManager::CreateDirectory(data_path + "/slam_data")) {
        return false;
    }

    key_frames_path_ = data_path + "/slam_data/key_frames";
    trajectory_path_ = data_path + "/slam_data/trajectory";

    if (!FileManager::CreateDirectory(key_frames_path_))
        return false;
    if (!FileManager::CreateDirectory(trajectory_path_))
        return false;

    if (!FileManager::CreateFile(ground_truth_ofs_, trajectory_path_ + "/ground_truth.txt"))
        return false;
    if (!FileManager::CreateFile(laser_odom_ofs_, trajectory_path_ + "/laser_odom.txt"))
        return false;

    return true;
}

bool BackEnd::Update(const CloudData& cloud_data, const PoseData& laser_odom, const PoseData &gnss_pose) {
    ResetParam();

    SaveTrajectory(laser_odom, gnss_pose);

    if (MaybeNewKeyFrame(cloud_data, laser_odom)) {
        AddNodeAndEdge(gnss_pose);
        MaybeOptimized();
    }

    return true;
}

void BackEnd::ResetParam() {
    has_new_key_frame_ = false;
    has_new_optimized_ = false;
}

bool BackEnd::SaveTrajectory(const PoseData& laser_odom, const PoseData& gnss_pose) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ground_truth_ofs_ << gnss_pose.pose(i, j);
            laser_odom_ofs_ << laser_odom.pose(i, j);
            
            if (i == 2 && j == 3) {
                ground_truth_ofs_ << std::endl;
                laser_odom_ofs_ << std::endl;
            } else {
                ground_truth_ofs_ << " ";
                laser_odom_ofs_ << " ";
            }
        }
    }

    return true;
}

bool BackEnd::MaybeNewKeyFrame(const CloudData& cloud_data, const PoseData& laser_odom) {
    static Eigen::Matrix4f last_key_pose = laser_odom.pose;
    if (key_frames_deque_.size() == 0) {
        has_new_key_frame_ = true;
        last_key_pose = laser_odom.pose;
    } else if (fabs(laser_odom.pose(0,3) - last_key_pose(0,3)) + 
        fabs(laser_odom.pose(1,3) - last_key_pose(1,3)) +
        fabs(laser_odom.pose(2,3) - last_key_pose(2,3)) > key_frame_distance_) {

        has_new_key_frame_ = true;
        last_key_pose = laser_odom.pose;
    }

    if (has_new_key_frame_) {
        // 把关键帧点云存储到硬盘里
        std::string file_path = key_frames_path_ + "/key_frame_" + std::to_string(key_frames_deque_.size()) + ".pcd";
        pcl::io::savePCDFileBinary(file_path, *cloud_data.cloud_ptr);

        KeyFrame key_frame;
        key_frame.time = laser_odom.time;
        key_frame.index = (unsigned int)key_frames_deque_.size();
        key_frame.pose = laser_odom.pose;
        key_frames_deque_.push_back(key_frame);

        current_key_frame_ = key_frame;
    }
    return has_new_key_frame_;
}

bool BackEnd::AddNodeAndEdge(const PoseData& gnss_data) {
    Eigen::Isometry3d isometry;
    // Add key frame as vertex
    isometry.matrix() = current_key_frame_.pose.cast<double>();
    graph_optimizer_ptr_->AddSe3Node(isometry, false);
    new_key_frame_cnt_++;

    // Add odometry as edges
    static KeyFrame last_key_frame = current_key_frame_;
    int node_num = graph_optimizer_ptr_->GetNodeNum();
    if (node_num > 1) {
        Eigen::MatrixX4f relative_pose = last_key_frame.pose.inverse() * current_key_frame_.pose;
        isometry.matrix() = relative_pose.cast<double>();
        graph_optimizer_ptr_->AddSe3Edge(node_num-2, node_num-1, isometry, graph_optimizer_config_.odom_edge_noise);
    }
    last_key_frame = current_key_frame_;

    // Add gnss pose as prior
    if (graph_optimizer_config_.use_gnss) {
        Eigen::Vector3d xyz(static_cast<double>(gnss_data.pose(0,3)),
                            static_cast<double>(gnss_data.pose(1,3)),
                            static_cast<double>(gnss_data.pose(2,3)));
        graph_optimizer_ptr_->AddSe3PriorXYZEdge(node_num-1, xyz, graph_optimizer_config_.gnss_noise);
        new_gnss_cnt_++;
    }
    return true;
}

bool BackEnd::MaybeOptimized() {
    bool need_optimize = false; 

    if (new_gnss_cnt_ >= graph_optimizer_config_.optimize_step_with_gnss)
        need_optimize = true;
    if (new_loop_cnt_ >= graph_optimizer_config_.optimize_step_with_loop)
        need_optimize = true;
    if (new_key_frame_cnt_ >= graph_optimizer_config_.optimize_step_with_key_frame)
        need_optimize = true;

    if (!need_optimize)
        return false;
    
    new_gnss_cnt_ = 0;
    new_loop_cnt_ = 0;
    new_key_frame_cnt_ = 0;

    if (graph_optimizer_ptr_->Optimize())
        has_new_optimized_ = true;

    return true;
}

bool BackEnd::ForceOptimize() {
    if (graph_optimizer_ptr_->Optimize())
        has_new_optimized_ = true;
    return has_new_optimized_;
}

void BackEnd::GetOptimzedKeyFrames(std::deque<KeyFrame> &key_frames_deque) {
    key_frames_deque.clear();
    if (graph_optimizer_ptr_->GetNodeNum() > 0) {
        std::deque<Eigen::Matrix4f> optimized_pose;
        graph_optimizer_ptr_->GetOptimizedPose(optimized_pose);
        KeyFrame key_frame;
        for (size_t i = 0; i < optimized_pose.size(); ++i) {
            key_frame.pose = optimized_pose.at(i);
            key_frame.index = (unsigned int)i;
            key_frames_deque.push_back(key_frame);
        }
    }
}

bool BackEnd::HasNewKeyFrame() {
    return has_new_key_frame_;
}

bool BackEnd::HasNewOptimized() {
    return has_new_optimized_;
}

void BackEnd::GetLatestKeyFrame(KeyFrame  &key_frame) {
    key_frame = current_key_frame_;
}
} //namespace myslam
