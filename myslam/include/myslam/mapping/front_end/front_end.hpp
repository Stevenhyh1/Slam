#ifndef MYSLAM_MAPPING_FRONT_END_FRONT_END_HPP_
#define MYSLAM_MAPPING_FRONT_END_FRONT_END_HPP_

#include <deque>
#include <iostream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>

#include "myslam/sensor_data/cloud_data.hpp"
#include "myslam/models/cloud_filter/voxel_filter.hpp"
#include "myslam/models/registration/ndt_registration.hpp"
#include "myslam/sensor_data/cloud_data.hpp"
#include "myslam/sensor_data/gnss_data.hpp"
#include "myslam/sensor_data/imu_data.hpp"
#include "myslam/sensor_data/velocity_data.hpp"


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
    FrontEnd();

    // Lidar Odometry
    bool SetInitPose(const Eigen::Matrix4f& init_pose);
    bool Update(const CloudData& cloud_data, Eigen::Matrix4f &cloud_pose);
    void UpdateNewFrame(const Frame& new_key_frame);

private:
    // Directory to save map
    std::string data_path_ = "";

    // Current data
    Frame current_frame_;
    Eigen::Matrix4f lidar_odometry_ = Eigen::Matrix4f::Identity();

    // Filters and Pointcloud registration methods
    std::shared_ptr<CloudFilterInterface> current_frame_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
    std::shared_ptr<RegistrationInterface> registration_ptr_;

    // Local Map(Sliding window)
    int local_frame_num;
    float key_frame_distance;
    std::deque<Frame> local_map_frames_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_ptr_;

    // Initialize filters and registrations from configuration
    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
    bool InitWithConfig();
    bool InitParam(const YAML::Node &node);
    bool InitDataPath(const YAML::Node &node);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &node);
    bool InitRegistration(std::shared_ptr<RegistrationInterface> &registration_ptr, const YAML::Node &node);

};
} // namespace myslam

#endif