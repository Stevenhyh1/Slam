#ifndef MYSLAM_MAPPING_VIEWER_VIEWER_FLOW_HPP_
#define MYSLAM_MAPPING_VIEWER_VIEWER_FLOW_HPP_

#include <deque>
#include <ros/ros.h>

#include "myslam/subscriber/cloud_subscriber.hpp"
#include "myslam/subscriber/odometry_subscriber.hpp"
#include "myslam/subscriber/key_frame_subscriber.hpp"
#include "myslam/subscriber/key_frames_subscriber.hpp"
#include "myslam/publisher/odometry_publisher.hpp"
#include "myslam/publisher/cloud_publisher.hpp"
#include "myslam/mapping/viewer/viewer.hpp"

namespace myslam {
class ViewerFlow {
public:
    ViewerFlow(ros::NodeHandle &nh);

    bool Run();
    bool SaveMap();

private:

    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> transformed_odom_sub_ptr_;
    std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
    std::shared_ptr<KeyFramesSubscriber> optimized_key_frames_sub_ptr_;

    std::shared_ptr<OdometryPublisher> optimized_odom_pub_ptr_;
    std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
    std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> local_map_pub_ptr_;

    std::shared_ptr<Viewer> viewer_ptr_;

    std::deque<CloudData> cloud_data_buff_;
    std::deque<PoseData> transformed_odom_buff_;
    std::deque<KeyFrame> key_frame_buff_;
    std::deque<KeyFrame> optimized_key_frames_;
    std::deque<KeyFrame> all_key_frames_;

    CloudData current_cloud_data_;
    PoseData current_transformed_odom_;

    bool ReadData();
    bool HasData();
    bool ValidData();
    bool PublishGlobalData();
    bool PublishLocalData();

};
} //namespace myslam

#endif