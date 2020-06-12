#ifndef MYSLAM_SENSOR_DATA_GNSS_DATA_HPP_
#define MYSLAM_SENSOR_DATA_GNSS_DATA_HPP_

#include <string>
#include <vector>

#include "GeographicLib/LocalCartesian.hpp"

namespace myslam {
class GNSSData {
public:
    double time = 0.0;
    double longitude = 0.0;
    double latitude = 0.0;
    double altitude = 0.0;
    double local_E = 0.0;
    double local_N = 0.0;
    double local_U = 0.0;
    int status = 0;
    int service = 0;

    void InitOriginPosition();
    void UpdateXYZ();

private:
    static GeographicLib::LocalCartesian geo_converter;
    static bool origin_position_inited;

};
} //namespace myslam

#endif