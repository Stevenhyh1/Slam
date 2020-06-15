#ifndef MYSLAM_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_
#define MYSLAM_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_

#include <queue>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include "myslam/sensor_data/velocity_data.hpp"

namespace myslam {
class VelocitySubscriber{

public:
    VelocitySubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
    VelocitySubscriber() = default;
    void ParseData(std::deque<VelocityData> & velocity_data_deque);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<VelocityData> new_velocity_data_;
    
    void msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

};
} //namespace myslam

#endif