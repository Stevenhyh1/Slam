#include "myslam/models/distortion_adjust/distortion_adjust.hpp"

namespace myslam {
void DistortionAdjust::SetMotionInfo(float scan_period, VelocityData velocity_data) {
    scan_period_ = scan_period;
    velocity_ << velocity_data.linear_velocity_.x, velocity_data.linear_velocity_.y, velocity_data.linear_velocity_.z;
    angular_rate_ << velocity_data.angular_velocity_.roll, velocity_data.angular_velocity_.pitch, velocity_data.angular_velocity_.yaw;
}

bool DistortionAdjust::AdjustCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud_ptr) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>(*input_cloud_ptr));
    output_cloud_ptr->points.clear();

    float orientation_space = 2.0 * M_PI;
    float delete_space = 5.0 * M_PI / 180.0; // 5 degree in radian
    float start_orientation = atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x);

    Eigen::AngleAxisf rotate_vector(start_orientation, Eigen::Vector3f::UnitZ()); // Rotate around z axis by theta
    Eigen::Matrix3f rotate_matrix = rotate_vector.toRotationMatrix(); // R_sb
    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
    transformation_matrix.block<3,3>(0, 0) = rotate_matrix.transpose(); // G_bs
    pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transformation_matrix); // points are wrt. rotated frame

    velocity_ = rotate_matrix.transpose() * velocity_;
    angular_rate_ = rotate_matrix.transpose() * angular_rate_;

    for (size_t i = 1; i < origin_cloud_ptr->points.size(); ++i) {
        float orientaion = atan2(origin_cloud_ptr->points[i].y, origin_cloud_ptr->points[i].x);
        if (orientaion < 0.0) {
            orientaion += 2.0 * M_PI;
        }

        if (orientaion < delete_space || 2.0 * M_PI - orientaion < delete_space) {
            continue;
        }

        float t = fabs(orientaion)/orientation_space * scan_period_ - scan_period_/2;

        Eigen::Vector3f origin_point(origin_cloud_ptr->points[i].x, origin_cloud_ptr->points[i].y, origin_cloud_ptr->points[i].z);

        Eigen::Matrix3f current_rotation = UpdataMatrix(t);
        Eigen::Vector3f current_translation = velocity_ * t;
        Eigen::Vector3f adjusted_point = current_rotation * origin_point + current_translation;
        pcl::PointXYZ point;
        point.x = adjusted_point(0);
        point.y = adjusted_point(1);
        point.z = adjusted_point(2);
        output_cloud_ptr->points.push_back(point);
    }

    pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transformation_matrix.inverse());
    return true;
}

Eigen::Matrix3f DistortionAdjust::UpdataMatrix(float real_time) {
    Eigen::Vector3f euler_angle = angular_rate_ * real_time; //ZYX Euler angle convention
    Eigen::AngleAxisf r_vz(euler_angle(2), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf r_vy(euler_angle(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf r_vx(euler_angle(0), Eigen::Vector3f::UnitX());
    Eigen::Matrix3f r = r_vz.toRotationMatrix() * r_vy.toRotationMatrix() * r_vz.toRotationMatrix();
    return r;
}

} //namespace myslam