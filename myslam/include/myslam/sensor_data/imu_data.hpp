#ifndef MYSLAM_SENSOR_DATA_IMU_DATA_HPP_
#define MYSLAM_SENSOR_DATA_IMU_DATA_HPP_

#include <Eigen/Dense>

namespace myslam {
class IMUData
{

public:
    IMUData():
        linear_acceleration({0,0,0}), 
        angular_velocity({0,0,0}),
        orientation({0,0,0,0})
        {}

    struct LinearAcceleration 
    {
        double x, y, z;
    };

    struct AngularVelocity
    {
        double x, y, z;
    };

    struct Orientation
    {
        double x, y, z, w;
    };

    double time = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    Orientation orientation;
    
    Eigen::Matrix3f GetOrientationMatrix() {
        Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
        Eigen::Matrix3f matrix = q.matrix().cast<float>();

        return matrix;
    }
    
};
} // namespace myslam


#endif