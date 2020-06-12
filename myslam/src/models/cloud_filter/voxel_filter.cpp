#include "myslam/models/cloud_filter/voxel_filter.hpp"

namespace myslam {

VoxelFilter::VoxelFilter(const YAML::Node& node) {
    float leaf_size_x = node["leaf_size"][0].as<float>();
    float leaf_size_y = node["leaf_size"][1].as<float>();
    float leaf_size_z = node["leaf_size"][2].as<float>();

    // std::cout << "Leaf sizes of voxel filter are: " << std::endl;
    // std::cout << "x: " << leaf_size_x << "  y:  " << leaf_size_y << "  z:  " << leaf_size_z << std::endl;
    
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

VoxelFilter::VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

bool VoxelFilter::Filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud_ptr) {
    // std::cout << "Setting input cloud" << std::endl;
    voxel_filter_.setInputCloud(input_cloud_ptr);
    // std::cout << "Filtering" << std::endl;
    voxel_filter_.filter(*filtered_cloud_ptr);
    // std::cout << "Filtered" << std::endl;
    return true;
}

bool VoxelFilter::SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);

    return true;
}
} //namespace myslam