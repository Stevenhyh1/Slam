#include "myslam/publisher/cloud_publisher.hpp"

namespace myslam {
CloudPublisher::CloudPublisher(ros::NodeHandle& nh, std::string topic_name, size_t buff_size, std::string frame_id)
    :nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
    }

void CloudPublisher::Publish(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_input) {
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);
    cloud_ptr_output->header.stamp = ros::Time::now();
    cloud_ptr_output->header.frame_id = frame_id_;
    publisher_.publish(*cloud_ptr_output);
    // ROS_INFO("Publish PointCloud");
}

void CloudPublisher::Publish(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_input, double time) {
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);

    ros::Time ros_time((float)time);
    cloud_ptr_output->header.stamp = ros_time;
    cloud_ptr_output->header.frame_id = frame_id_;
    publisher_.publish(*cloud_ptr_output);
}

bool CloudPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
} //namespace myslam