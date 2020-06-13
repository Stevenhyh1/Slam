#include "myslam/sensor_data/imu_data.hpp"

namespace myslam {

IMUData::IMUData():
    linear_acceleration({0,0,0}), 
    angular_velocity({0,0,0}),
    orientation({0,0,0,0}){}

Eigen::Matrix3f IMUData::GetOrientationMatrix() {
    Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
    Eigen::Matrix3f matrix = q.matrix().cast<float>();

    return matrix;
}

bool IMUData::SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time) {

    while (UnsyncedData.size() >= 2) {
        double first_time = UnsyncedData.front().time, second_time = UnsyncedData[1].time;
        if (first_time > sync_time) {
            return false;
        }
        if (second_time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }
        if (sync_time - first_time > 0.2) {
            UnsyncedData.pop_front();
            return false;
        }
        if (second_time - sync_time > 0.2) {
            UnsyncedData.pop_front();
            return false;
        }
        break;
    }

    if (UnsyncedData.size() < 2) {
        return false;
    }

    IMUData first_data = UnsyncedData.front();
    IMUData second_data = UnsyncedData[1];
    IMUData synced_data;

    double front_scale = (sync_time - first_data.time) / (second_data.time - first_data.time);
    double back_scale = (second_data.time - sync_time) / (second_data.time - first_data.time);

    synced_data.time = sync_time;
    synced_data.linear_acceleration.x = first_data.linear_acceleration.x * front_scale + second_data.linear_acceleration.x * back_scale;
    synced_data.linear_acceleration.y = first_data.linear_acceleration.y * front_scale + second_data.linear_acceleration.y * back_scale;
    synced_data.linear_acceleration.z = first_data.linear_acceleration.z * front_scale + second_data.linear_acceleration.z * back_scale;
    synced_data.angular_velocity.x = first_data.angular_velocity.x * front_scale + second_data.angular_velocity.x * back_scale;
    synced_data.angular_velocity.y = first_data.angular_velocity.y * front_scale + second_data.angular_velocity.y * back_scale;
    synced_data.angular_velocity.z = first_data.angular_velocity.z * front_scale + second_data.angular_velocity.z * back_scale;
    synced_data.orientation.x = first_data.orientation.x * front_scale + second_data.orientation.x * back_scale;
    synced_data.orientation.y = first_data.orientation.y * front_scale + second_data.orientation.y * back_scale;
    synced_data.orientation.z = first_data.orientation.z * front_scale + second_data.orientation.z * back_scale;
    synced_data.orientation.w = first_data.orientation.w * front_scale + second_data.orientation.w * back_scale;
    synced_data.orientation.Normalize();

    SyncedData.push_back(synced_data);

    return true;
}
} //namespace myslam