#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "utils.h"
#include "camera.h"
#include "frame.h"

namespace myvo
{
    class Frame;
    class MapPoint
    {
        public:
            typedef std::shared_ptr<MapPoint> Ptr;
            unsigned long id_;
            Eigen::Vector3d pos_;
            Eigen::Vector3d norm_;
            cv::Mat descriptor_;
            int observed_times_;
            int correct_times_;

            MapPoint();
            MapPoint(long id, Eigen::Vector3d position, Eigen::Vector3d norm);

            static MapPoints::Ptr createMapPoint();
    };
}

#endif