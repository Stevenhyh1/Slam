#ifndef MYSLAM_SUBSCRIBER_IMU_SUBSCRIBER_HPP_
#define MYSLAM_SUBSCRIBER_IMU_SUBSCRIBER_HPP_

#include <deque>
#include <string>

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"

#include "myslam/sensor_data/imu_data.hpp"

namespace myslam{
class IMUSubscriber
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<IMUData> new_imu_data_;

public:
    IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    void ParseData(std::deque<IMUData>& deque_imu_data);

private:
    void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

};
}// namespace myslam

#endif