#ifndef MYSLAM_SUBSCRIBER_KEY_FRAMES_SUBSCRIBER_HPP_
#define MYSLAM_SUBSCRIBER_KEY_FRAMES_SUBSCRIBER_HPP_

#include <queue>
#include <string>

#include "Eigen/Dense"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include "myslam/sensor_data/key_frame.hpp"

namespace myslam {
class KeyFramesSubscriber
{
public:
    KeyFramesSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
    KeyFramesSubscriber() = default;

    bool ParseData(std::deque<KeyFrame> &key_frame_buff);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<KeyFrame> new_key_frame_;
    
    void msg_callback(const nav_msgs::PathConstPtr &path_msg_ptr);
};
}
#endif