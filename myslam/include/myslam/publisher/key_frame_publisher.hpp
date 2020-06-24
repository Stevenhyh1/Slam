#ifndef MYSLAM_PUBLISHER_KEY_FRAME_PUBLISHER_HPP_
#define MYSLAM_PUBLISHER_KEY_FRAME_PUBLISHER_HPP_

#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "myslam/sensor_data/key_frame.hpp"

namespace myslam {
class KeyFramePublisher {
public:
    KeyFramePublisher(ros::NodeHandle &nh, std::string topic_name, std::string frame_id, size_t buff_size);

    void Publish(KeyFrame &key_frame);

    bool HasSubscribers();

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    geometry_msgs::PoseStamped pose_;

}; 
}

#endif