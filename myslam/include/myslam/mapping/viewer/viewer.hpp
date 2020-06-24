#ifndef MYSLAM_MAPPING_VIEWER_VIEWER_HPP_
#define MYSLAM_MAPPING_VIEWER_VIEWER_HPP_

#include <deque>
#include <string>

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <yaml-cpp/yaml.h>

#include <myslam/sensor_data/cloud_data.hpp>
#include <myslam/sensor_data/key_frame.hpp>
#include <myslam/sensor_data/pose_data.hpp>
#include <myslam/models/cloud_filter/voxel_filter.hpp>

namespace myslam {
class Viewer
{
private:

public:
    Viewer();

    bool UpdateWithOptimizedKeyFrames(std::deque<KeyFrame> &optimized_key_frames);
    bool UpdateWithNewKeyFrame(std::deque<KeyFrame> &new_key_frames, PoseData transformed_data, CloudData cloud_data);

    bool Savemap();
    bool GetCurrentPose(Eigen::Matrix4f &current_pose);
    bool GetCurrentScan(pcl::PointCloud<pcl::PointXYZ>::Ptr &current_scan);
    bool GetLocalMap(pcl::PointCloud<pcl::PointXYZ>::Ptr &local_map_ptr);
    bool GetGlobalMap (pcl::PointCloud<pcl::PointXYZ>::Ptr &global_map_ptr);
    bool HasNewLocalMap();
    bool HasNewGlobalMap();

private:
    
    std::string data_path_ = "";
    std::string key_frames_path_ = "";
    std::string map_path_ = "";
    int local_frame_nums_;

    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;

    Eigen::Matrix4f origin_to_optimize_ = Eigen::Matrix4f::Identity();
    PoseData optimized_odom_;
    CloudData optimized_cloud_;
    std::deque<KeyFrame> optimized_key_frames_;
    std::deque<KeyFrame> all_key_frames_;

    bool new_global_map_ = false;
    bool new_local_map_ = false;
    
    bool InitWithConfig();
    bool InitParam(const YAML::Node &config_node);
    bool InitDataPath(const YAML::Node &config_node);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_pointer, const YAML::Node &config_node);

    bool OptimizeKeyFrames();
    bool JoinLocalMap(pcl::PointCloud<pcl::PointXYZ>::Ptr &local_map_ptr);
    bool JoinGlobalMap(pcl::PointCloud<pcl::PointXYZ>::Ptr &global_map_ptr);
    bool JoinCloudMap(const std::deque<KeyFrame> &key_frames, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr);

};
}//namespace myslam

#endif