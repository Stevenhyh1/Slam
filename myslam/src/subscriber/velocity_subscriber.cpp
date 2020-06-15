#include "myslam/subscriber/velocity_subscriber.hpp"

namespace myslam {

VelocitySubscriber::VelocitySubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size): nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &VelocitySubscriber::msg_callback, this);
}

void VelocitySubscriber::msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr) {
    VelocityData velocity_data;
    velocity_data.time = twist_msg_ptr->header.stamp.toSec();

    velocity_data.linear_velocity_.x = twist_msg_ptr->twist.linear.x;
    velocity_data.linear_velocity_.y = twist_msg_ptr->twist.linear.y;
    velocity_data.linear_velocity_.z = twist_msg_ptr->twist.linear.z;

    velocity_data.angular_velocity_.roll = twist_msg_ptr->twist.angular.x;
    velocity_data.angular_velocity_.pitch = twist_msg_ptr->twist.angular.y;
    velocity_data.angular_velocity_.yaw = twist_msg_ptr->twist.angular.z;

    new_velocity_data_.push_back(velocity_data);
}

void VelocitySubscriber::ParseData(std::deque<VelocityData> & velocity_data_deque) {
    if (!new_velocity_data_.empty()) {
        velocity_data_deque.insert(velocity_data_deque.end(),new_velocity_data_.begin(), new_velocity_data_.end());
        new_velocity_data_.clear();
    }
}

} //namespace myslam
