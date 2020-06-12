#ifndef MYSLAM_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_
#define MYSLAM_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_

#include <deque>

#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"

#include "myslam/sensor_data/gnss_data.hpp"

namespace myslam{
class GNSSSubscriber
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<GNSSData> new_gnss_data_;

public:
    GNSSSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
    GNSSSubscriber() = default;
    void ParseData(std::deque<GNSSData>& deque_gnss_data);

private:
    void msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);
};
} //namespace myslam

#endif