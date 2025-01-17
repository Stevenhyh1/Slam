#ifndef MYSLAM_PUBLISHER_LOOP_POSE_PUBLISHER_HPP_
#define MYSLAM_PUBLISHER_LOOP_POSE_PUBLISHER_HPP_

#include <string> 

#include <ros/ros.h>

#include <myslam/sensor_data/loop_pose.hpp>

namespace myslam {

class LoopPosePublisher{
    public:
        LoopPosePublisher(ros::NodeHandle &nh, std::string topic_name, std::string frame_id, size_t buff_size);
        LoopPosePublisher() = default;

        void Publish(LoopPose &loop_pose);
        bool HasSubscribers();

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_ = "";
};

}//myslam

#endif