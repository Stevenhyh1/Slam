#ifndef MYSLAM_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_
#define MYSLAM_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_

#include <queue>
#include <string>

#include "Eigen/Dense"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "myslam/sensor_data/pose_data.hpp"

namespace myslam {
class OdometrySubscriber
{
public:
    OdometrySubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
    OdometrySubscriber() = default;

    bool ParseData(std::deque<PoseData> &pose_buff);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<PoseData> new_pose_;
    
    void msg_callback(const nav_msgs::OdometryConstPtr &odometry_msg_ptr);
};
}

#endif