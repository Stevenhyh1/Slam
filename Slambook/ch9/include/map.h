#ifndef MAP_H
#define MAP_H

#include "utils.h"
#include "camera.h"
#include "frame.h"
#include "mappoint.h"

namespace myvo
{
    class Map 
    {
        public: 
        typedef std::shared_ptr<Map> Ptr;
        unordered_map<unsigned long, MapPoint::Ptr> map_points_;
        unordered_map<unsigend long, Frame::Ptr> keyframes_;

        Map() {}

        void insertKeyFrame( Frame::Ptr frame);
        void insertMapPoint( MapPoint::Ptr map_point);
    };
}

#endif