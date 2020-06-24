#include <ros/ros.h>
#include "glog/logging.h"

#include "myslam/data_preprocess/data_preprocess_flow.hpp"
// #include "myslam/global_definition.hpp"

const std::string WORK_SPACE_PATH = "/home/yihe/catkin_ws/src/Slam/myslam";

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "data_pretreat_node");
    ros::NodeHandle nh;

    std::shared_ptr<myslam::DataPreprocessFlow> data_pretreat_flow_ptr = std::make_shared<myslam::DataPreprocessFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        data_pretreat_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}
