#include "myslam/sensor_data/velocity_data.hpp"

namespace myslam {
bool VelocityData::SyncData(std::deque<VelocityData> &UnsyncedData, std::deque<VelocityData> &SyncData, double sync_time) {
    while (UnsyncedData.size() >= 2){
        double first_time = UnsyncedData.front().time;
        double second_time = UnsyncedData[1].time;
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
    
    VelocityData front_data = UnsyncedData.front();
    VelocityData back_data = UnsyncedData[1];
    VelocityData sync_data;

    double front_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    double back_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);

    sync_data.time = sync_time;
    sync_data.linear_velocity_.x = front_data.linear_velocity_.x * front_scale + back_data.linear_velocity_.x * back_scale;
    sync_data.linear_velocity_.y = front_data.linear_velocity_.y * front_scale + back_data.linear_velocity_.y * back_scale;
    sync_data.linear_velocity_.z = front_data.linear_velocity_.z * front_scale + back_data.linear_velocity_.z * back_scale;
    sync_data.angular_velocity_.roll = front_data.angular_velocity_.roll * front_scale + back_data.angular_velocity_.roll * back_scale;
    sync_data.angular_velocity_.pitch = front_data.angular_velocity_.pitch * front_scale + back_data.angular_velocity_.pitch * back_scale;
    sync_data.angular_velocity_.yaw = front_data.angular_velocity_.yaw * front_scale + back_data.angular_velocity_.yaw * back_scale;

    SyncData.push_back(sync_data);

    return true;
}

void VelocityData::TransformCoordinate(Eigen::Matrix4f transformation_matrix) {
    // Transform velocity from body frame to spatial frame: V_s = Adg_sb * V_b
    // Transformation matrix: G_sb
    Eigen::Matrix4d G = transformation_matrix.cast<double>();
    Eigen::Matrix3d R = G.block<3,3>(0,0);
    Eigen::Vector3d p = G.block<3,1>(0,3);
    Eigen::Vector3d v(linear_velocity_.x, linear_velocity_.y, linear_velocity_.z);
    Eigen::Vector3d w(angular_velocity_.roll, angular_velocity_.pitch, angular_velocity_.yaw);

    w = R * w;
    v = p.cross(w) + R * v;

    linear_velocity_.x = v(0);
    linear_velocity_.y = v(1);
    linear_velocity_.z = v(2);
    angular_velocity_.roll = w(1);
    angular_velocity_.pitch = w(2);
    angular_velocity_.yaw = w(3);
}
} //namespace myslam
