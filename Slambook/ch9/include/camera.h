#ifndef CAMERA_H
#define CMAERA_H

#include "utils.h"

namespace myvo 
{

    class Camera {
        private:
            float fx_, fy_, cx_, cy_ , depth_;

        public:
            typedef std::shared_ptr<Camera> Ptr;
            Camera();
            Camera(float fx, float fy, float cx, float cy, float depth):
                   fx_(fx), fy_(fy), cx_(cx), cy_(cy), depth_(depth) {}

            // coordinate transform: world, camera, pixel
            Eigen::Vector3d world2camera( const Eigen::Vector3d& p_w, const Sophus::SE3d& T_c_w );
            Eigen::Vector3d camera2world( const Eigen::Vector3d& p_c, const Sophus::SE3d& T_c_w );
            Eigen::Vector2d camera2pixel( const Eigen::Vector3d& p_c );
            Eigen::Vector3d pixel2camera( const Eigen::Vector2d& p_p, double depth=1 ); 
            Eigen::Vector3d pixel2world ( const Eigen::Vector2d& p_p, const Sophus::SE3d& T_c_w, double depth=1 );
            Eigen::Vector2d world2pixel ( const Eigen::Vector3d& p_w, const Sophus::SE3d& T_c_w );

    };

}

#endif