#ifndef MYSLAM_MAPPING_LOOP_CLOSURE_LOOP_CLOSURE_HPP_
#define MYSLAM_MAPPING_LOOP_CLOSURE_LOOP_CLOSURE_HPP_

#include <deque>
#include <string>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <yaml-cpp/yaml.h>

#include "myslam/sensor_data/key_frame.hpp"
#include "myslam/sensor_data/loop_pose.hpp"
#include "myslam/models/registration/registration_interface.hpp"
#include "myslam/models/cloud_filter/cloud_filter_interface.hpp"

namespace myslam{
class LoopClosing{
    public:
        LoopClosing();

        bool Update(const KeyFrame keyframe, const KeyFrame key_gnss);

        bool HasNewLoopPose();
        LoopPose& GetCurrentLoopPose();
    
    private:
        std::string key_frames_path_ = "";
        int extend_frame_num_ = 3;
        int loop_step_ = 10;
        int diff_num_ = 100;
        float detect_area_ = 10.0;
        float fitness_score_limit_ = 2.0;

        std::shared_ptr<CloudFilterInterface> scan_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> map_filter_ptr_;
        std::shared_ptr<RegistrationInterface> registration_ptr_;

        std::deque<KeyFrame> all_key_frames_;
        std::deque<KeyFrame> all_key_gnss_;

        LoopPose current_loop_pose_;
        bool has_new_loop_pose_ = false;

        bool InitwithConfig();
        bool InitParam(const YAML::Node &config_node);
        bool InitDataPath(const YAML::Node &config_node);
        bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node &config_node);
        bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node);

        bool DetectNearestKeyFrame(int &key_frame_index);
        bool CloudRegistration(int key_frame_index);
        bool JointMap(int key_frame_index, pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_ptr, Eigen::Matrix4f &map_pose);
        bool JointScan(pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud_ptr, Eigen::Matrix4f &scan_pose);
        bool Registration(pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_ptr,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud_ptr,
                          Eigen::Matrix4f& scan_pose,
                          Eigen::Matrix4f& result_pose);

};
}//namespace myslam

#endif