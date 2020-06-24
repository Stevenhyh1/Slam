#ifndef MYSLAM_PUBLISHER_ODOMETRY_PUBLISHER_HPP_
#define MYSLAM_PUBLISHER_ODOMETRY_PUBLISHER_HPP_

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace myslam {
class OdometryPublisher
{
private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    nav_msgs::Odometry odometry_;

public:
    OdometryPublisher(ros::NodeHandle& nh, std::string topic_name, std::string base_frame_id, std::string child_frame_id, int buff_size);
    OdometryPublisher() = default;
    void Publish(const Eigen::Matrix4f& transform_matrix);
    void Publish(const Eigen::Matrix4f& transform_matrix, double time);
};
} //namespace myslam
#endif