#include "myslam/sensor_data/gnss_data.hpp"

#include "glog/logging.h"
#include <iostream>

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

bool GNSSData::SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time) {

    // std::cout << "In GNSS data synchronization" << std::endl;
    while (UnsyncedData.size() >= 2) {
        double first_time = UnsyncedData.front().time, second_time = UnsyncedData[1].time;
        // std::cout << "Point cloud time: " << sync_time << std::endl;
        // std::cout << "Front time: " << sync_time - first_time << std::endl;
        // std::cout << "Back time: " << sync_time - second_time << std::endl;
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

    GNSSData first_data = UnsyncedData.front();
    GNSSData second_data = UnsyncedData[1];
    GNSSData synced_data;

    double front_scale = (sync_time - first_data.time) / (second_data.time - first_data.time);
    double back_scale = (second_data.time - sync_time) / (second_data.time - first_data.time);

    synced_data.time = sync_time;
    synced_data.status = second_data.status;
    synced_data.longitude = first_data.longitude * front_scale + second_data.longitude * back_scale;
    synced_data.latitude = first_data.latitude * front_scale + second_data.latitude * back_scale;
    synced_data.altitude = first_data.altitude * front_scale + second_data.altitude * back_scale;
    synced_data.local_E = first_data.local_E * front_scale + second_data.local_E * back_scale;
    synced_data.local_N = first_data.local_N * front_scale + second_data.local_N * back_scale;
    synced_data.local_U = first_data.local_U * front_scale + second_data.local_U * back_scale;

    SyncedData.push_back(synced_data);
    return true;
}

}