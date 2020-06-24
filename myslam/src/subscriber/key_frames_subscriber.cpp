#include "myslam/subscriber/key_frames_subscriber.hpp"

namespace myslam {

KeyFramesSubscriber::KeyFramesSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size):nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &KeyFramesSubscriber::msg_callback, this);
}

void KeyFramesSubscriber::msg_callback(const nav_msgs::PathConstPtr &path_msg_ptr) {
    
    new_key_frame_.clear();

    for (size_t i=0; i<path_msg_ptr->poses.size(); ++i) {
        
        KeyFrame key_frame;

        key_frame.time = path_msg_ptr->poses.at(i).header.stamp.toSec();
        key_frame.index = path_msg_ptr->poses.at(i).header.seq;

        key_frame.pose(0,3) = path_msg_ptr->poses.at(i).pose.position.x;
        key_frame.pose(1,3) = path_msg_ptr->poses.at(i).pose.position.y;
        key_frame.pose(2,3) = path_msg_ptr->poses.at(i).pose.position.z;

        Eigen::Quaternionf q(path_msg_ptr->poses.at(i).pose.orientation.w, 
                            path_msg_ptr->poses.at(i).pose.orientation.x, 
                            path_msg_ptr->poses.at(i).pose.orientation.y,
                            path_msg_ptr->poses.at(i).pose.orientation.z);
        key_frame.pose.block<3,3>(0,0) = q.toRotationMatrix();

        new_key_frame_.push_back(key_frame);
    }

}

bool KeyFramesSubscriber::ParseData(std::deque<KeyFrame> &key_frame_buff) {
    if (!new_key_frame_.empty()) {
        key_frame_buff = new_key_frame_;
        new_key_frame_.clear();
    }
    return true;
}

}