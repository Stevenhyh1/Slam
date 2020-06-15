#include "myslam/front_end/front_end.hpp"

#include <cmath>
#include <iostream>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include "glog/logging.h"

namespace myslam{
FrontEnd::FrontEnd(ros::NodeHandle &nh)
    :local_map_ptr_(new pcl::PointCloud<pcl::PointXYZ>),
    global_map_ptr_(new pcl::PointCloud<pcl::PointXYZ>),
    result_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZ>){

    cloud_sub_ptr_ = std::make_shared<CloudSubscriber> (nh, "/kitti/velo/pointcloud", 10);
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber> (nh, "/kitti/oxts/gps/fix", 10);
    imu_sub_ptr_ = std::make_shared<IMUSubscriber> (nh, "/kitti/oxts/imu", 10);
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber> (nh, "/kitti/oxts/gps/vel", 10);
    lidar_to_imu_ptr_ = std::make_shared<TFListener> (nh, "velo_link", "imu_link");

    cloud_pub_ptr_ = std::make_shared<CloudPublisher> (nh, "current_scan", 100, "/map");
    local_map_pub_ptr_ = std::make_shared<CloudPublisher> (nh, "local_map", 100, "/map");
    global_map_pub_ptr_ = std::make_shared<CloudPublisher> (nh, "global_map", 100, "/map");
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher> (nh, "laser_odom", "map", "lidar", 100);
    gnss_pub_ptr_ = std::make_shared<OdometryPublisher> (nh, "gnss", "map", "lidar", 100);

    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust> ();

    // Default parameters
    InitWithConfig();
    }

bool FrontEnd::InitWithConfig() {
    std::string config_file_path = "/home/yihe/catkin_ws/src/Slam/myslam/config/front_end/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    InitDataPath(config_node);
    InitParam(config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    InitFilter("frame", current_frame_filter_ptr_, config_node);
    InitFilter("display", displayer_filter_ptr_, config_node);
    InitRegistration(registration_ptr_, config_node);

    return true;
}

bool FrontEnd::InitParam(const YAML::Node &node) {
    local_frame_num = node["local_frame_num"].as<int>();
    key_frame_distance = node["key_frame_distance"].as<float>();

    return true;
}

bool FrontEnd::InitDataPath(const YAML::Node &node) {
    data_path_ = node["data_path"].as<std::string> ();
    
    if (boost::filesystem::is_directory(data_path_)) {
        boost::filesystem::remove_all(data_path_);
    }

    boost::filesystem::create_directory(data_path_);
    if (!boost::filesystem::is_directory(data_path_)) {
        std::cout << "Failed to Create Folder" << data_path_ << std::endl;
        return false;
    } else {
        std::cout << "Directory of saved maps " << data_path_ << "created!" << std::endl;
    }

    std::string key_frame_path = data_path_ + "/key_frames";
    boost::filesystem::create_directory(key_frame_path);
    if (!boost::filesystem::is_directory(key_frame_path)) {
        std::cout << "Failed to Create Folder" << key_frame_path << std::endl;
        return false;
    } else {
        std::cout << "Directory of save key frames " << key_frame_path << "created!" << std::endl;
    }

    return true;
}

bool FrontEnd::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &node) {
    std::string filter_method  = node[filter_user + "_filter"].as<std::string>();
    
    if (filter_method == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(node[filter_method][filter_user]);

    } else {
        std::cout << "Cannot find configurations for " << filter_method << std::endl;
        return false;
    }

    return true;
}

bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface> &registration_ptr, const YAML::Node &node) {
    std::string registration_method = node["registration_method"].as<std::string>();
    
    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(node[registration_method]);
    } else {
        std::cout << "Cannot find configurations for" << registration_method << std::endl;
        return false;
    }

    return true;
}

bool FrontEnd::Update(const CloudData& cloud_data, Eigen::Matrix4f &cloud_pose) {

    // std::cout << "In update: " << std::endl;
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
    
    // Save key frame to the disk
    std::string file_path = data_path_ + "/key_frames/key_frame_" + std::to_string(global_map_frames_.size()) + ".pcd";
    if (new_key_frame.cloud_data.cloud_ptr->size() > 0) {
        pcl::io::savePCDFileBinary(file_path, *new_key_frame.cloud_data.cloud_ptr);
    }

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
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, 
                                 *transformed_cloud_ptr, 
                                 local_map_frames_.at(i).pose);
        *local_map_ptr_ += *transformed_cloud_ptr;
    }
    has_new_local_map_ = true;

    //Update ndt matching targets
    if (local_map_frames_.size() < 10) {
        registration_ptr_->setInputTarget(local_map_ptr_);
    } else {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_local_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
        registration_ptr_->setInputTarget(filtered_local_map_ptr);
    }

    //Update global map
    global_map_frames_.push_back(key_frame);
}

bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;
    return true;
}

bool FrontEnd::GetNewLocalMap(pcl::PointCloud<pcl::PointXYZ>::Ptr &local_map_ptr) {
    if (has_new_local_map_) {
        displayer_filter_ptr_->Filter(local_map_ptr_, local_map_ptr);
        return true;
    }
    return false;
}

bool FrontEnd::GetNewGlobalMap(pcl::PointCloud<pcl::PointXYZ>::Ptr &global_map_ptr) {
    if (has_new_global_map_) {
        has_new_global_map_ = false;
        displayer_filter_ptr_->Filter(global_map_ptr_, global_map_ptr);
        global_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        return true;
    }
    return false;
}

bool FrontEnd::GetCurrentScan(pcl::PointCloud<pcl::PointXYZ>::Ptr &current_scan_ptr) {
    displayer_filter_ptr_->Filter(result_cloud_ptr_, current_scan_ptr);
    return true;
}

bool FrontEnd::ReadData() {

    // std::cout << "Reading point cloud data" << std::endl;
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    // gnss_sub_ptr_->ParseData(gnss_data_buff_);
    // imu_sub_ptr_->ParseData(imu_data_buff_);
    if (cloud_data_buff_.empty()) {
        return false;
    }
    double sync_time = cloud_data_buff_.front().time;

    static std::deque<GNSSData> unsynced_gnss_data_;
    static std::deque<IMUData> unsynced_imu_data_;
    static std::deque<VelocityData> unsynced_velocity_data_;

    gnss_sub_ptr_->ParseData(unsynced_gnss_data_);
    imu_sub_ptr_->ParseData(unsynced_imu_data_);
    velocity_sub_ptr_->ParseData(unsynced_velocity_data_);

    bool valid_gnss = GNSSData::SyncData(unsynced_gnss_data_, gnss_data_buff_, sync_time);
    bool valid_imu = IMUData::SyncData(unsynced_imu_data_, imu_data_buff_, sync_time);
    bool valid_velocity = VelocityData::SyncData(unsynced_velocity_data_, velocity_data_buff_, sync_time);

    // if (!valid_gnss) std::cout << "No valid gnss" << std::endl;
    // if (!valid_imu) std::cout << "No valid imu" << std::endl;

    static bool sensor_inited = false;
    if (!sensor_inited) {
        if (!valid_gnss || !valid_imu || !valid_velocity) {
            cloud_data_buff_.pop_front();
            // std::cout << "No valid interpolation" << std::endl;
            return false;
        }
        sensor_inited = true;
    }
    // std::cout << "Valid interpolation" << std::endl;
    return true;
}

bool FrontEnd::InitCalibration() {
    
    static bool calibration_received = false;
    if (!calibration_received) {
        lidar_to_imu_ptr_->LookupData(lidar_to_imu);
        calibration_received = true;
        std::cout << "Initializing calibration" << std::endl;
    }
    return true;
}

bool FrontEnd::InitGNSS() {
    
    static bool gnss_inited = false;
    if (!gnss_inited && !gnss_data_buff_.empty()) {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
        std::cout << "Initializing GNSS" << std::endl;
    }
    return true;
}

bool FrontEnd::HasData() {
    return (!cloud_data_buff_.empty() && !gnss_data_buff_.empty() && !imu_data_buff_.empty());
}

bool FrontEnd::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_velocity_data_ = velocity_data_buff_.front();

    double time_diff = current_cloud_data_.time - current_imu_data_.time;
    if (time_diff < -0.05) {
        // Point cloud is much earlier than imu
        cloud_data_buff_.pop_front();
        return false;

    } else if (time_diff > 0.05) {
        // Point cloud is much later than imu
        gnss_data_buff_.pop_front();
        imu_data_buff_.pop_front();
        velocity_data_buff_.pop_front();
        return false;
    } 
    // Coming approximately the same time
    cloud_data_buff_.pop_front();
    gnss_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    velocity_data_buff_.pop_front();
    return true;
}

bool FrontEnd::UpdateGNSSOdometry() {

    gnss_odometry_ = Eigen::Matrix4f::Identity();

    current_gnss_data_.UpdateXYZ();
    gnss_odometry_(0,3) = current_gnss_data_.local_E;
    gnss_odometry_(1,3) = current_gnss_data_.local_N;
    gnss_odometry_(2,3) = current_gnss_data_.local_U;
    gnss_odometry_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();
    gnss_odometry_ *= lidar_to_imu;

    return true;
}

bool FrontEnd::UpdataLaserOdometry() {

    // std::cout << "Starting laser odometry" << std::endl;
    current_velocity_data_.TransformCoordinate(lidar_to_imu);
    distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);
    
    lidar_odometry_ = Eigen::Matrix4f::Identity();

    static bool front_end_pose_init = false;
    if (!front_end_pose_init) {
        lidar_odometry_ = gnss_odometry_;
        SetInitPose(gnss_odometry_);
        front_end_pose_init = true;
        return true;
    } 

    if (Update(current_cloud_data_, lidar_odometry_)) {
        return true;
    };
        return false;

}

bool FrontEnd::PublishData() {

    gnss_pub_ptr_->Publish(gnss_odometry_);
    laser_odom_pub_ptr_->Publish(lidar_odometry_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_scan(new pcl::PointCloud<pcl::PointXYZ>);
    GetCurrentScan(current_scan);
    cloud_pub_ptr_->Publish(result_cloud_ptr_);

    if (GetNewLocalMap(local_map_ptr_)) {
        local_map_pub_ptr_->Publish(local_map_ptr_);
    }

    // if (GetNewGlobalMap(global_map_ptr_)) {
    //     std::cout << "Publishing Global map" << std::endl;
    //     global_map_pub_ptr_->Publish(global_map_ptr_);
    // }
}

bool FrontEnd::SaveMap() {
    global_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    std::string key_frame_path = "";
    pcl::PointCloud<pcl::PointXYZ>::Ptr key_frmae_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i=0; i<global_map_frames_.size(); ++i) {
        key_frame_path = data_path_ + "/key_frames/key_frame_" + std::to_string(i) + ".pcd";
        pcl::io::loadPCDFile(key_frame_path, *key_frmae_cloud_ptr);
        pcl::transformPointCloud(*key_frmae_cloud_ptr, 
                                 *transformed_cloud_ptr,
                                  global_map_frames_.at(i).pose);
        *global_map_ptr_ += *transformed_cloud_ptr;
    }

    std::string map_file_path = data_path_ + "map.pcd";
    pcl::io::savePCDFileBinary(map_file_path, *global_map_ptr_);
    has_new_global_map_ = true;

    return true;
}

bool FrontEnd::SaveTrajectory() {
    static std::string trajectory_path = data_path_ + "/trajectories";
    static std::string gnss_file_name = trajectory_path + "/gnss_data.txt";
    static std::string odom_file_name = trajectory_path + "/odom_data.txt";
    static bool is_file_created = false;
    static std::ofstream gnss_file, odom_file;

    if (!is_file_created) {
        if (!FileManager::CreateDirectory(trajectory_path)) {
            return false;
        }
        if (!FileManager::CreateFile(gnss_file, gnss_file_name)) {
            return false;
        }
        if (!FileManager::CreateFile(odom_file, odom_file_name)) {
            return false;
        }
        is_file_created = true;
    }

    for (int i=0; i<3; ++i) {
        for (int j=0; j<4; ++j) {
            // std::cout << "writing to file" << std::endl;
            gnss_file << gnss_odometry_(i, j);
            odom_file << lidar_odometry_(i, j);
            // if reaches the end of the transformation matrix
            if (i == 2 && j == 3) {
                gnss_file << std::endl;
                odom_file << std::endl;
            } else {
                gnss_file << " ";
                odom_file << " ";
            }
        }
    }

    return true;

}

bool FrontEnd::Run() {
    if (!InitCalibration()) {
        return false;
    }

    if (!ReadData()) {
        return false;
    }
    // ReadData();

    if (!InitGNSS()) {
        return false;
    }
    
    while (HasData())
    {
        if (!ValidData()) {
            continue;
        }
        UpdateGNSSOdometry();
        if (UpdataLaserOdometry()) {
            PublishData();
            SaveTrajectory();
        }
    }
    return true;
}

} // namespace myslam