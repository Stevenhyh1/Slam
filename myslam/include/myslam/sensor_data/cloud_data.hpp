#ifndef MYSLAM_SENSOR_DATA_CLOUD_DATA_HPP_
#define MYSLAM_SENSOR_DATA_CLOUD_DATA_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace myslam{
class CloudData {
public:
    CloudData():cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>) {}
    
    double time = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
    
};
} // namespace myslam

#endif