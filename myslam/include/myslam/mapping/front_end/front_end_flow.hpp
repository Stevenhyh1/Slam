#ifndef MYSLAM_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_
#define MYSLAM_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_

#include "Eigen/Dense"
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <myslam/subscriber/cloud_subscriber.hpp>
#include <myslam/subscriber/odometry_subscriber.hpp>
#include <myslam/publisher/odometry_publisher.hpp>
#include <myslam/mapping/front_end/front_end.hpp>

namespace myslam {
class FrontEndFlow {
public:
    FrontEndFlow(ros::NodeHandle &nh);

    bool Run();

private:
    
    ros::NodeHandle nh_;

    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_;
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
    std::shared_ptr<FrontEnd> front_end_ptr_;

    std::deque<CloudData> cloud_data_buff_;
    std::deque<PoseData> pose_data_buff_;

    CloudData current_scan_;
    Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
    
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool UpdateLaserOdometry();
    bool PublishData();
};
}// namespace myslam

#endif