#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include "glog/logging.h"

#include "myslam/subscriber/cloud_subscriber.hpp"
#include "myslam/subscriber/gnss_subscriber.hpp"
#include "myslam/subscriber/imu_subscriber.hpp"
#include "myslam/publisher/cloud_publisher.hpp"
#include "myslam/publisher/odometry_publisher.hpp"
#include "myslam/tf_listener/tf_listener.hpp"
#include "myslam/front_end/front_end.hpp"

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    std::shared_ptr<myslam::FrontEnd> front_end_ptr = std::make_shared<myslam::FrontEnd>(nh);

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        front_end_ptr->Run();

        rate.sleep();
    }

    return 0;
}