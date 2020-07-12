#include <ros/ros.h>
#include <glog/logging.h>

#include "myslam/mapping/loop_closure/loop_closure_flow.hpp"

const std::string WORK_SPACE_PATH = "/home/yihe/catkin_ws/src/Slam/myslam";

using namespace myslam;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "loop_closure_node");
    ros::NodeHandle nh;

    std::shared_ptr<LoopClosingFlow> loop_closing_flow_ptr = std::make_shared<LoopClosingFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        loop_closing_flow_ptr->Run();

        rate.sleep();
    }
    return 0;
}