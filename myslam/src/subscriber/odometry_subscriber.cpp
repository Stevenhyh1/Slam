#include "myslam/subscriber/odometry_subscriber.hpp"

namespace myslam {

OdometrySubscriber::OdometrySubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size):nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &OdometrySubscriber::msg_callback, this);
}

void OdometrySubscriber::msg_callback(const nav_msgs::OdometryConstPtr &odometry_msg_ptr) {
    
    PoseData pose_data;

    pose_data.time = odometry_msg_ptr->header.stamp.toSec();

    pose_data.pose(0,3) = odometry_msg_ptr->pose.pose.position.x;
    pose_data.pose(1,3) = odometry_msg_ptr->pose.pose.position.y;
    pose_data.pose(2,3) = odometry_msg_ptr->pose.pose.position.z;

    Eigen::Quaternionf q(odometry_msg_ptr->pose.pose.orientation.w, 
                         odometry_msg_ptr->pose.pose.orientation.x, 
                         odometry_msg_ptr->pose.pose.orientation.y,
                         odometry_msg_ptr->pose.pose.orientation.z);
    pose_data.pose.block<3,3>(0,0) = q.toRotationMatrix();

    new_pose_.push_back(pose_data);
}

bool OdometrySubscriber::ParseData(std::deque<PoseData> &pose_buff) {
    if (!new_pose_.empty()) {
        pose_buff.insert(pose_buff.end(), new_pose_.begin(), new_pose_.end());
        new_pose_.clear();
    }
    return true;
}

}