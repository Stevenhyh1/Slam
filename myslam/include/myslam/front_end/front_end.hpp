#ifndef MYSLAM_FRONT_END_FRONT_END_HPP_
#define MYSLAM_FRONT_END_FRONT_END_HPP_

#include <deque>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>

#include "myslam/sensor_data/cloud_data.hpp"
#include "myslam/models/cloud_filter/voxel_filter.hpp"
#include "myslam/models/registration/ndt_registration.hpp"
#include "myslam/sensor_data/cloud_data.hpp"
#include "myslam/sensor_data/gnss_data.hpp"
#include "myslam/sensor_data/imu_data.hpp"
#include "myslam/subscriber/cloud_subscriber.hpp"
#include "myslam/subscriber/gnss_subscriber.hpp"
#include "myslam/subscriber/imu_subscriber.hpp"
#include "myslam/tf_listener/tf_listener.hpp"
#include "myslam/publisher/cloud_publisher.hpp"
#include "myslam/publisher/odometry_publisher.hpp"

namespace myslam {
class FrontEnd
{
public:
    
    // Frame class
    class Frame
    {
    public:
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        CloudData cloud_data;
    };

    // Constructor
    FrontEnd(ros::NodeHandle &nh);

    // Interfaces
    bool Run();
    bool SaveMap();
    bool PublishGlobalMap();
    bool GetNewLocalMap(pcl::PointCloud<pcl::PointXYZ>::Ptr &local_map_ptr);
    bool GetNewGlobalMap(pcl::PointCloud<pcl::PointXYZ>::Ptr &global_map_ptr);
    bool GetCurrentScan(pcl::PointCloud<pcl::PointXYZ>::Ptr &current_scan_ptr);

private:
    // Directory to save map
    std::string data_path_ = "";

    // Subscribers
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<TFListener> lidar_to_imu_ptr_;

    // Publishers
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;

    // Buffer to store data
    std::deque<CloudData> cloud_data_buff_;
    std::deque<GNSSData> gnss_data_buff_;
    std::deque<IMUData> imu_data_buff_;

    // Messege to publish
    Eigen::Matrix4f gnss_odometry_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f lidar_odometry_ = Eigen::Matrix4f::Identity();

    // Filters and Pointcloud registration methods
    std::shared_ptr<CloudFilterInterface> current_frame_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> displayer_filter_ptr_;
    std::shared_ptr<RegistrationInterface> registration_ptr_;

    // Local Map(Sliding window) and Global Map
    bool has_new_local_map_ = false;
    bool has_new_global_map_ = false;
    int local_frame_num;
    float key_frame_distance;
    std::deque<Frame> local_map_frames_;
    std::deque<Frame> global_map_frames_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud_ptr_;

    // Current Data
    Frame current_frame_;
    CloudData current_cloud_data_;
    GNSSData current_gnss_data_;
    IMUData current_imu_data_;
    Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();  

    // Initialize filters and registrations from configuration
    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
    bool InitWithConfig();
    bool InitParam(const YAML::Node &node);
    bool InitDataPath(const YAML::Node &node);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &node);
    bool InitRegistration(std::shared_ptr<RegistrationInterface> &registration_ptr, const YAML::Node &node);

    // Lidar odometry
    bool SetInitPose(const Eigen::Matrix4f& init_pose);
    bool Update(const CloudData& cloud_data, Eigen::Matrix4f &cloud_pose);
    void UpdateNewFrame(const Frame& new_key_frame);
    
    // Processing flow function
    bool ReadData();
    bool InitCalibration();
    bool InitGNSS();
    bool HasData();
    bool ValidData();
    bool UpdateGNSSOdometry();
    bool UpdataLaserOdometry();
    bool PublishData();
};
} // namespace myslam

#endif