#include "myslam/publisher/key_frame_publisher.hpp"

namespace myslam {

KeyFramePublisher::KeyFramePublisher(ros::NodeHandle &nh, std::string topic_name, std::string frame_id, size_t buff_size): nh_(nh) {
    publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_name, buff_size);
    pose_.header.frame_id = frame_id;
}

void KeyFramePublisher::Publish(KeyFrame &key_frame) {
    
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

    publisher_.publish(pose_);

}

bool KeyFramePublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
} //namespace myslam