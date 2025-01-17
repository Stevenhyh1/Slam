#ifndef MYSLAM_MAPPING_BACK_END_BACK_END_FLOW_HPP_
#define MSSLAM_MAPPING_BACK_END_BACK_END_FLOW_HPP_

#include <ros/ros.h>

#include "myslam/subscriber/cloud_subscriber.hpp"
#include "myslam/subscriber/odometry_subscriber.hpp"
#include "myslam/publisher/odometry_publisher.hpp"
#include "myslam/publisher/key_frame_publisher.hpp"
#include "myslam/publisher/key_frames_publisher.hpp"
#include "myslam/mapping/back_end/back_end.hpp"

namespace myslam {
class BackEndFlow {
  public:
    BackEndFlow(ros::NodeHandle& nh);

    bool Run();

    bool ForceOptimize();

  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool UpdateBackEnd();
    bool SaveTrajectory();
    bool PublishData();

  private:
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> gnss_pose_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> laser_odom_sub_ptr_;

    std::shared_ptr<OdometryPublisher> transformed_odom_pub_ptr_;
    std::shared_ptr<KeyFramePublisher> key_frame_pub_ptr_;
    std::shared_ptr<KeyFramePublisher> key_gnss_pub_ptr_;
    std::shared_ptr<KeyFramesPublisher> key_frames_pub_ptr_;
    std::shared_ptr<BackEnd> back_end_ptr_;

    std::deque<CloudData> cloud_data_buff_;
    std::deque<PoseData> gnss_pose_data_buff_;
    std::deque<PoseData> laser_odom_data_buff_;

    PoseData current_gnss_pose_data_;
    PoseData current_laser_odom_data_;
    CloudData current_cloud_data_;
};
}//namespace myslam

#endif