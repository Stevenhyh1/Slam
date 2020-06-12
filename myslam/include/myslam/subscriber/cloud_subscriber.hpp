#ifndef MYSLAM_SUBSCRIBER_CLOUD_SUBSCRIBER_H_
#define MYSLAM_SUBSCRIBER_CLOUD_SUBSCRIBER_H_

#include <deque>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "myslam/sensor_data/cloud_data.hpp"

namespace myslam {
class CloudSubscriber {
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<CloudData> new_cloud_data_;

public:
    CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    CloudSubscriber() = default;
    void ParseData(std::deque<CloudData>& deque_cloud_data);

private:
    void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);
};
} // namespace myslam

#endif