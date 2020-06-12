#ifndef MYSLAM_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_
#define MYSLAM_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

namespace myslam{
class CloudFilterInterface
{
public:
    virtual ~CloudFilterInterface() = default;
    
    virtual bool Filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud_ptr) = 0;
};
} //namespace myslam


#endif