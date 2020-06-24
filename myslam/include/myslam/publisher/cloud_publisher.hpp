#ifndef MYSLAM_PUBLISHER_CLOUD_PUBLISHER_HPP_
#define MYSLAM_PUBLISHER_CLOUD_PUBLISHER_HPP_

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "myslam/sensor_data/cloud_data.hpp"

namespace myslam {
class CloudPublisher
{
private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;

public:
    CloudPublisher(ros::NodeHandle& nh, std::string topic_name, size_t buff_size, std::string frame_id);
    CloudPublisher() = default;
    void Publish(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_input);
    void Publish(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_input, double time);
    bool HasSubscribers();
};
} //namespace myslam

#endif