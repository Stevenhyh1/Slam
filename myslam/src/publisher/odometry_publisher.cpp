#include "glog/logging.h"

#include "myslam/publisher/odometry_publisher.hpp"


namespace myslam {
OdometryPublisher::OdometryPublisher(ros::NodeHandle& nh, std::string topic_name, std::string base_frame_id, std::string child_frame_id, int buff_size) 
:nh_(nh) {
    publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
    odometry_.header.frame_id = base_frame_id;
    odometry_.child_frame_id = child_frame_id;
}
void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix) {
    odometry_.header.stamp = ros::Time::now();

    odometry_.pose.pose.position.x = transform_matrix(0,3);
    odometry_.pose.pose.position.y = transform_matrix(1,3);
    odometry_.pose.pose.position.z = transform_matrix(2,3);

    Eigen::Quaternionf q;
    q = transform_matrix.block<3,3>(0,0);
    odometry_.pose.pose.orientation.x = q.x();
    odometry_.pose.pose.orientation.y = q.y();
    odometry_.pose.pose.orientation.z = q.z();
    odometry_.pose.pose.orientation.w = q.w();

    publisher_.publish(odometry_);
    // LOG(INFO) << "Odometry matrix is : " << transform_matrix << std::endl;
}

void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix, double time) {
    ros::Time ros_time((float)time);
    odometry_.header.stamp = ros_time;

    odometry_.pose.pose.position.x = transform_matrix(0,3);
    odometry_.pose.pose.position.y = transform_matrix(1,3);
    odometry_.pose.pose.position.z = transform_matrix(2,3);

    Eigen::Quaternionf q;
    q = transform_matrix.block<3,3>(0,0);
    odometry_.pose.pose.orientation.x = q.x();
    odometry_.pose.pose.orientation.y = q.y();
    odometry_.pose.pose.orientation.z = q.z();
    odometry_.pose.pose.orientation.w = q.w();

    publisher_.publish(odometry_);
    // ROS_INFO("Publish odometry");
}
} //namespace myslam