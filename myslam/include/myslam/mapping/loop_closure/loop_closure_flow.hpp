#ifndef MYSLAM_MAPPING_LOOP_CLOSURE_LOOP_CLOSURE_FLOW_HPP_
#define MYSLAM_MAPPING_LOOP_CLOSURE_LOOP_CLOSURE_FLOW_HPP_

#include <deque>
#include <string>

#include <ros/ros.h>

#include "myslam/subscriber/key_frame_subscriber.hpp"
#include "myslam/publisher/loop_pose_publisher.hpp"
#include "myslam/mapping/loop_closure/loop_closure.hpp"

namespace myslam {
class LoopClosingFlow {
    public:
        LoopClosingFlow(ros::NodeHandle &nh);
        bool Run();
    private:
        std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
        std::shared_ptr<KeyFrameSubscriber> key_gnss_sub_ptr_;

        std::shared_ptr<LoopPosePublisher> loop_pose_pub_ptr_;
        std::shared_ptr<LoopClosing> loop_closing_ptr_;

        std::deque<KeyFrame> key_frame_buff_;
        std::deque<KeyFrame> key_gnss_buff_;

        KeyFrame current_key_frame_;
        KeyFrame current_key_gnss_;
        
        bool ReadData();
        bool HasData();
        bool ValidData();
        bool PublishData();
};
} //namespace myslam

#endif