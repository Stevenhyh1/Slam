#include "myslam/subscriber/loop_pose_subscriber.hpp"

#include "glog/logging.h"

#include "myslam/sensor_data/loop_pose.hpp"

namespace myslam{
LoopPoseSubscriber::LoopPoseSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size):nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &LoopPoseSubscriber::msg_callback, this);
}

void LoopPoseSubscriber::ParseData(std::deque<LoopPose> loop_pose_deque) {
    if (new_loop_pose_.size() > 0) {
        loop_pose_deque.insert(loop_pose_deque.end(), new_loop_pose_.begin(), new_loop_pose_.end());
        new_loop_pose_.clear();
    }
}

void LoopPoseSubscriber::msg_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &loop_pose_msg_ptr) {
    LoopPose loop_pose;

    loop_pose.time = loop_pose_msg_ptr->header.stamp.toSec();
    loop_pose.index0 = (unsigned int)loop_pose_msg_ptr->pose.covariance[0];
    loop_pose.index1 = (unsigned int)loop_pose_msg_ptr->pose.covariance[1];

    loop_pose.pose(0,3) = loop_pose_msg_ptr->pose.pose.position.x;
    loop_pose.pose(1,3) = loop_pose_msg_ptr->pose.pose.position.y;
    loop_pose.pose(2,3) = loop_pose_msg_ptr->pose.pose.position.z;

    Eigen::Quaternionf q;
    q.x() = loop_pose_msg_ptr->pose.pose.orientation.x;
    q.y() = loop_pose_msg_ptr->pose.pose.orientation.y;
    q.z() = loop_pose_msg_ptr->pose.pose.orientation.z;
    q.w() = loop_pose_msg_ptr->pose.pose.orientation.w;

    loop_pose.pose.block<3,3>(0,0) = q.toRotationMatrix();

    new_loop_pose_.push_back(loop_pose);
}

}//namespace myslam