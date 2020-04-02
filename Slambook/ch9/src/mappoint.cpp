#include "mappoint.h"

namespace myvo
{
    MapPoint::MapPoint():
    id_(0), pos_(Eigen::Vector3d (0,0,0)), norm_(Eigen::Vector3d (0,0,0)), oberserved_times_(0), correct_times_(0) {}

    MapPoint::MapPoint(long id, Eigen::Vector3d position, Eigen::Vector3d norm):
    id_(0), pos_(position), norm_(norm), observed_times_(0), correct_times_(0) {}

    MapPoint::Ptr createMapPoint() {
        static long factory_id = 0;
        return MapPoint::Ptr( 
            new MapPoint( factory_id++, Vector3d(0,0,0), Vector3d(0,0,0) )
        );
    }
}