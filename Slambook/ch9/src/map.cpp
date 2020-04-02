#include "map.h"

namespace myvo 
{
    Map::Map(): {}

    void Map::insertKeyFrame (Frame::Ptr frame) {
        std::cout << "Key Frame Size: " << keyframe_.size() << std::endl;
        keyframe_[frame->id_] = frame;
    }

    void Map::insertMapPoint (MapPoint::Ptr map_point) {
        map_points_[map_point -> id_] = map_point
    }
}