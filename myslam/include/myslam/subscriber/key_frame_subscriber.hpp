#ifndef MYSLAM_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_HPP_
#define MYSLAM_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_HPP_

#include <queue>
#include <string>

#include "Eigen/Dense"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "myslam/sensor_data/key_frame.hpp"

namespace myslam {
class KeyFrameSubscriber
{
public:
    KeyFrameSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
    KeyFrameSubscriber() = default;

    bool ParseData(std::deque<KeyFrame> &key_frame_buff);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<KeyFrame> new_key_frame_;
    
    void msg_callback(const geometry_msgs::PoseStampedConstPtr &key_frame_msg_ptr);
};
}
#endif