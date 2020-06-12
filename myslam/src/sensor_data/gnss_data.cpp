#include "myslam/sensor_data/gnss_data.hpp"

#include "glog/logging.h"

bool myslam::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian myslam::GNSSData::geo_converter;

namespace myslam {

void GNSSData::InitOriginPosition() {
    geo_converter.Reset(latitude, longitude, altitude);
    origin_position_inited = true;
}

void GNSSData::UpdateXYZ() {
    if (!origin_position_inited) {
        LOG(WARNING) << "GeoConverter Has Not Set Origin Position";
    }
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}

}