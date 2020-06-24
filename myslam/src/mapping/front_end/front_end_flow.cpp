#include "glog/logging.h"

#include "myslam/mapping/front_end/front_end_flow.hpp"

namespace myslam {
    
FrontEndFlow::FrontEndFlow(ros::NodeHandle &nh): nh_(nh) {
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber> (nh_, "/synced_cloud", 10);
    gnss_sub_ptr_ = std::make_shared<OdometrySubscriber> (nh_, "/synced_gnss", 10);
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher> (nh_, "/laser_odom", "/map", "/lidar", 10);
    front_end_ptr_ = std::make_shared<FrontEnd>();
}


bool FrontEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    if (pose_data_buff_.empty())
        gnss_sub_ptr_->ParseData(pose_data_buff_);
    return true;
}

bool FrontEndFlow::HasData() {
    return (!cloud_data_buff_.empty()) && (!pose_data_buff_.empty());
}

bool FrontEndFlow::ValidData() {
    current_scan_ = cloud_data_buff_.front();
    cloud_data_buff_.pop_front();
    return true;
}

bool FrontEndFlow::UpdateLaserOdometry() {

    static bool laser_odom_inited = false;
    if (!laser_odom_inited) {
        front_end_ptr_->SetInitPose(pose_data_buff_.front().pose);
        laser_odom_inited = true;
        return front_end_ptr_->Update(current_scan_, laser_odometry_);
    }

    return front_end_ptr_->Update(current_scan_, laser_odometry_);
}

bool FrontEndFlow::PublishData() {
    // LOG(INFO) << "Publishing lidar odometry: " << std::endl;
    // LOG(INFO) << laser_odometry_.block<3,1>(0,3);
    laser_odom_pub_ptr_->Publish(laser_odometry_);
    return true;
}

bool FrontEndFlow::Run() {
    if (!ReadData()) return false;
    
    while (HasData()) {
        if (!ValidData()) continue;

        if (UpdateLaserOdometry()) {
            PublishData();
        }
    }
    
    return true;
}
}// namespace myslam