#include <ros/ros.h>
#include "glog/logging.h"

#include "myslam/mapping/viewer/viewer_flow.hpp"

using namespace myslam;

const std::string WORK_SPACE_PATH = "/home/yihe/catkin_ws/src/Slam/myslam";

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "viewer_node");
    ros::NodeHandle nh;

    std::shared_ptr<ViewerFlow> viewer_flow_ptr = std::make_shared<ViewerFlow>(nh);

    ros::Rate rate(100);

    while (ros::ok)
    {
        ros::spinOnce();
        
        viewer_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}