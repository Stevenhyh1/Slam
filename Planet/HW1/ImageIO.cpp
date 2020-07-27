#include <stdio.h>

#include <iostream>
#include <string>
#include <algorithm>
#include <boost/filesystem.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

int main(int argc, char *argv[]) {
    std::string LOAD_DIR = "/media/yihe/HDD/Data/TUM/rgbd_dataset_freiburg1_desk/rgb";
    std::string SAVE_DIR = "/media/yihe/HDD/Data/TUM/rgbd_dataset_freiburg1_desk/rgb_sequence";

    if (!boost::filesystem::is_directory(SAVE_DIR)) {
        std::cout << "Creating Directory: " << std::endl;
        boost::filesystem::create_directory(SAVE_DIR);
    }

    std::vector<cv::String> file_name;
    cv::glob(LOAD_DIR + "/*.png", file_name, false);
    std::sort(file_name.begin(), file_name.end());

    for (int i=0; i<file_name.size(); ++i) {
        cv::Mat image;
        image = cv::imread(file_name[i]);
        char new_name[100];
        sprintf(new_name, "/%04d.png", i);
        cv::imwrite(SAVE_DIR + new_name, image);
        // std::cout << SAVE_DIR + new_name << std::endl;
    }
}
