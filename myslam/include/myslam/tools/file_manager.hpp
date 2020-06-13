#ifndef MYSLAM_TOOLS_FILE_MANAGER_HPP_
#define MYSLAM_TOOLS_FILE_MANAGER_HPP_

#include <iostream>
#include <string>
#include <boost/filesystem.hpp>

namespace myslam {
class FileManager
{
public:
    FileManager() = default;
    
    static bool CreateDirectory(std::string directory_path);
    static bool CreateFile(std::ofstream &out_stream, std::string file_name);
};
}// namespace myslam

#endif