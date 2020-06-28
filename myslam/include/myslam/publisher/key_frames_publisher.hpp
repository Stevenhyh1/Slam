#ifndef MYSLAM_PUBLISHER_KEY_FRAMES_PUBLISHER_HPP_
#define MYSLAM_PUBLISHER_KEY_FRAMES_PUBLISHER_HPP_

#include <deque>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include "myslam/sensor_data/key_frame.hpp"

namespace myslam {
class KeyFramesPublisher {
public:
    KeyFramesPublisher(ros::NodeHandle &nh, std::string topic_name, std::string frame_id, size_t buff_size);

    void Publish(std::deque<KeyFrame> key_frames);

    bool HasSubscribers();

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;

}; 
}

#endif