#ifndef MYSLAM_SENSOR_DATA_VELOCITY_DATA_HPP_
#define MYSLAM_SENSOR_DATA_VELOCITY_DATA_HPP_

#include <queue>
#include <Eigen/Dense>

namespace myslam {
class VelocityData
{
public:
    VelocityData() = default;

    struct LinearVelocity
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct AngularVelocity
    {
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
    };

    double time = 0.0;
    LinearVelocity linear_velocity_;
    AngularVelocity angular_velocity_;

    static bool SyncData(std::deque<VelocityData> &UnsyncedData, std::deque<VelocityData> &SyncData, double sync_time);
    void TransformCoordinate(Eigen::Matrix4f transformation_matrix);
    
};
} //namespace myslam

#endif