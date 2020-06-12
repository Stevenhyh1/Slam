#ifndef MYSLAM_MODELS_CLOUD_FILTER_VOXEL_FILTER_HPP_
#define MYSLAM_MODELS_CLOUD_FILTER_VOXEL_FILTER_HPP_

#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>

#include "myslam/models/cloud_filter/cloud_filter_interface.hpp"

namespace myslam {
class VoxelFilter : public CloudFilterInterface{
private:

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;

public:
    
    VoxelFilter(const YAML::Node& node);
    VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    bool Filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud_ptr) override;

private:
    bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

};
} //namespace myslam

#endif