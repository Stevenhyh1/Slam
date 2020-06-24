#ifndef MYSLAM_DATA_PREPROCESS_DATA_PREPROCESS_FLOW_HPP_
#define MYSLAM_DATA_PREPROCESS_DATA_PREPROCESS_FLOW_HPP_

#include <ros/ros.h>

// Sensor data
#include "myslam/sensor_data/cloud_data.hpp"
#include "myslam/sensor_data/gnss_data.hpp"
#include "myslam/sensor_data/imu_data.hpp"
#include "myslam/sensor_data/velocity_data.hpp"
// Subscriber
#include "myslam/subscriber/cloud_subscriber.hpp"
#include "myslam/subscriber/gnss_subscriber.hpp"
#include "myslam/subscriber/imu_subscriber.hpp"
#include "myslam/subscriber/velocity_subscriber.hpp"
#include "myslam/tf_listener/tf_listener.hpp"
// Publisher
#include "myslam/publisher/cloud_publisher.hpp"
#include "myslam/publisher/odometry_publisher.hpp"
// Models
#include "myslam/models/distortion_adjust/distortion_adjust.hpp"

namespace myslam {
class DataPreprocessFlow
{
public:

    // Constructor
    DataPreprocessFlow(ros::NodeHandle &nh);

    // Interfaces
    bool Run();


private:

    // Subscribers
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
    std::shared_ptr<TFListener> lidar_to_imu_ptr_;

    // Publishers
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;

    // Filters and Pointcloud registration methods
    std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;

    // Buffer to store data
    std::deque<CloudData> cloud_data_buff_;
    std::deque<GNSSData> gnss_data_buff_;
    std::deque<IMUData> imu_data_buff_;
    std::deque<VelocityData> velocity_data_buff_;

    // Current Data
    CloudData current_cloud_data_;
    CloudData transformed_cloud_data_;
    GNSSData current_gnss_data_;
    IMUData current_imu_data_;
    VelocityData current_velocity_data_;
    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity(); 

    // Messege to publish
    Eigen::Matrix4f gnss_odometry_ = Eigen::Matrix4f::Identity();

    // Processing flow function
    bool ReadData(); // Reads data and synchronizes data based on point cloud 
    bool InitCalibration(); // Reads the transform between lidar and imu and saves it
    bool InitGNSS(); // Sets the initial position of GNSS
    bool HasData(); // Determines whether data is read
    bool ValidData(); // Checks whether data is synchronized
    bool TransformData(); // Adjusts distortion
    bool PublishData(); // Publishes data
};    
} //namespace myslam

#endif