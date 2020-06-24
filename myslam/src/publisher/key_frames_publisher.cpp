#include "myslam/publisher/key_frames_publisher.hpp"

namespace myslam {

KeyFramesPublisher::KeyFramesPublisher(ros::NodeHandle &nh, std::string topic_name, std::string frame_id, size_t buff_size): nh_(nh) {
    publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_name, buff_size);
    path_.header.frame_id = frame_id;
}

void KeyFramesPublisher::Publish(std::deque<KeyFrame> key_frames) {
    
    path_.header.stamp = ros::Time::now();

    for (size_t i=0; i<key_frames.size(); ++i) {
        KeyFrame key_frame = key_frames.at(i);
        geometry_msgs::PoseStamped pose_;

        ros::Time ros_time((float)key_frame.time);
        pose_.header.stamp = ros_time;
        pose_.header.seq = key_frame.index;
        
        pose_.pose.position.x = key_frame.pose(0,3);
        pose_.pose.position.y = key_frame.pose(1,3);
        pose_.pose.position.z = key_frame.pose(2,3);

        Eigen::Quaternionf q = key_frame.GetQuaternion();
        pose_.pose.orientation.w = q.w();
        pose_.pose.orientation.x = q.x();
        pose_.pose.orientation.y = q.y();
        pose_.pose.orientation.z = q.z();

        path_.poses.push_back(pose_);
    }

    publisher_.publish(path_);
}

bool KeyFramesPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
} //namespace myslam