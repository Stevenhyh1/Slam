#ifndef MYSLAM_SENSOR_DATA_IMU_DATA_HPP_
#define MYSLAM_SENSOR_DATA_IMU_DATA_HPP_

#include <cmath>
#include <queue>
#include <Eigen/Dense>

namespace myslam {
class IMUData
{
public:
    IMUData();

    struct LinearAcceleration 
    {
        double x, y, z;
    };

    struct AngularVelocity
    {
        double x, y, z;
    };

    class Orientation
    {
    public:
        
        double x, y, z, w;

        void Normalize() {
            double norm = std::sqrt(x*x + y*y + z*z + w*w);
            x /= norm;
            y /= norm;
            z /= norm;
            w /= norm;
        }
    };

    double time = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    Orientation orientation;
    
    Eigen::Matrix3f GetOrientationMatrix();
    static bool SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time);
};
} // namespace myslam


#endif