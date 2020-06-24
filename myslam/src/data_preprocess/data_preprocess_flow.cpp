#include "myslam/data_preprocess/data_preprocess_flow.hpp"
#include "glog/logging.h"

namespace myslam {

DataPreprocessFlow::DataPreprocessFlow(ros::NodeHandle &nh){

    cloud_sub_ptr_ = std::make_shared<CloudSubscriber> (nh, "/kitti/velo/pointcloud", 10);
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber> (nh, "/kitti/oxts/gps/fix", 10);
    imu_sub_ptr_ = std::make_shared<IMUSubscriber> (nh, "/kitti/oxts/imu", 10);
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber> (nh, "/kitti/oxts/gps/vel", 10);
    lidar_to_imu_ptr_ = std::make_shared<TFListener> (nh, "/velo_link", "/imu_link");

    cloud_pub_ptr_ = std::make_shared<CloudPublisher> (nh, "/synced_cloud", 100, "/velo_link");
    gnss_pub_ptr_ = std::make_shared<OdometryPublisher> (nh, "/synced_gnss", "/map", "/velo_link", 100);

    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust> ();

    }

bool DataPreprocessFlow::ReadData() {

    cloud_sub_ptr_->ParseData(cloud_data_buff_);

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

    static bool sensor_inited = false;
    if (!sensor_inited) {
        if (!valid_gnss || !valid_imu || !valid_velocity) {
            cloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }
    return true;
}

bool DataPreprocessFlow::InitCalibration() {

    static bool calibration_received = false;
    if (!calibration_received) {
        lidar_to_imu_ptr_->LookupData(lidar_to_imu_);
        calibration_received = true;
        LOG(INFO) << "The transformation from Lidar to IMU is: " << std::endl
                  << lidar_to_imu_ << std::endl;
    }
    return true;

}

bool DataPreprocessFlow::InitGNSS() {

    static bool gnss_inited = false;
    if (!gnss_inited && !gnss_data_buff_.empty()) {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
        LOG(INFO) << "Initializing GNSS" << std::endl;
    }
    return true;

}

bool DataPreprocessFlow::HasData() {
    return (!cloud_data_buff_.empty() && !gnss_data_buff_.empty() && !imu_data_buff_.empty()) && !velocity_data_buff_.empty();
}

bool DataPreprocessFlow::ValidData() {

    current_cloud_data_ = cloud_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_velocity_data_ = velocity_data_buff_.front();

    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
    double diff_velocity_time = current_cloud_data_.time - current_velocity_data_.time;
    double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;

    // LOG(INFO) << "Cloud data comes at " << current_cloud_data_.time << "s" << std::endl;
    // LOG(INFO) << "Cloud data comes at " << current_gnss_data_.time << "s" << std::endl;
    // LOG(INFO) << "Cloud data comes at  " << current_imu_data_.time << "s" << std::endl;
    // LOG(INFO) << "Cloud data comes at  " << current_velocity_data_.time << "s" << std::endl;

    // Sensor messages come after the pointcloud
    if (diff_imu_time < -0.05 || diff_velocity_time < -0.05 || diff_gnss_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }
    // Sensor messages come too soon before the pointclodu
    if (diff_imu_time > 0.05) {
        imu_data_buff_.pop_front();
        return false;
    }
    if (diff_velocity_time > 0.05) {
        velocity_data_buff_.pop_front();
        return false;
    }
    if (diff_gnss_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    }

    // Coming approximately the same time
    cloud_data_buff_.pop_front();
    gnss_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    velocity_data_buff_.pop_front();
    return true;

}

bool DataPreprocessFlow::TransformData() {

    // Update GNSS Odometry
    gnss_odometry_ = Eigen::Matrix4f::Identity();

    current_gnss_data_.UpdateXYZ();
    gnss_odometry_(0,3) = current_gnss_data_.local_E;
    gnss_odometry_(1,3) = current_gnss_data_.local_N;
    gnss_odometry_(2,3) = current_gnss_data_.local_U;
    gnss_odometry_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();
    gnss_odometry_ *= lidar_to_imu_;

    // Adjust Distortion
    current_velocity_data_.TransformCoordinate(lidar_to_imu_);
    distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, transformed_cloud_data_.cloud_ptr);

    return true;
}

bool DataPreprocessFlow::PublishData() {
    // LOG(INFO) << "Publishing gnss odometry: " << std::endl;
    // LOG(INFO) << gnss_odometry_.block<3,1>(0,3);
    cloud_pub_ptr_->Publish(transformed_cloud_data_.cloud_ptr, current_cloud_data_.time);
    gnss_pub_ptr_->Publish(gnss_odometry_, current_gnss_data_.time);
}

bool DataPreprocessFlow::Run() {

    if (!ReadData()) {
        return false;
    }

    if (!InitCalibration()) {
        return false;
    }

    if (!InitGNSS()) {
        return false;
    }
    
    while (HasData())
    {
        if (!ValidData()) {
            continue;
        }
        TransformData();
        PublishData();
    }
    return true;
}
} //namespace myslam