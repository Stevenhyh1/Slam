#ifndef MYSLAM_TF_LISTENER_TF_LISTENER_HPP
#define MYSLAM_TF_LISTENER_TF_LISTENER_HPP

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace myslam {
class TFListener
{
private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    std::string base_frame_id_;
    std::string child_frame_id_;
public:
    TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id);
    TFListener() = default;
    bool LookupData(Eigen::Matrix4f& transform_matrix);
private:
    bool TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix);
};
} // namespace myslam

#endif