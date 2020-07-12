#ifndef MYSLAM_SUBSCRIBER_LOOP_POSE_SUBSCRIBER_HPP_
#define MYSLAM_SUBSCRIBER_LOOP_POSE_SUBSCRIBER_HPP_

#include <deque>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "myslam/sensor_data/loop_pose.hpp"

namespace myslam{
    class LoopPoseSubscriber {
        public:
            LoopPoseSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
            LoopPoseSubscriber() = default;
            void ParseData(std::deque<LoopPose> loop_pose_deque);
            
        private:
            ros::NodeHandle &nh_;
            ros::Subscriber subscriber_;
            std::deque<LoopPose> new_loop_pose_;
            
            void msg_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &loop_pose_msg_ptr);
    };
}//namespace myslam

#endif