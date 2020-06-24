#include <ros/ros.h>
#include "glog/logging.h"

// #include "myslam/global_definition.hpp"
#include "myslam/mapping/front_end/front_end_flow.hpp"

const std::string WORK_SPACE_PATH = "/home/yihe/catkin_ws/src/Slam/myslam";

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    std::shared_ptr<myslam::FrontEndFlow> front_end_ptr = std::make_shared<myslam::FrontEndFlow>(nh);

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        front_end_ptr->Run();

        rate.sleep();
    }

    return 0;
}