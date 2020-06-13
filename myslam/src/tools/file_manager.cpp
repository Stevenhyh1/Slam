#include "myslam/tools/file_manager.hpp"

namespace myslam{

bool FileManager::CreateDirectory(std::string directory_path){
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }
    if (!boost::filesystem::is_directory(directory_path)) {
        std::cout << "Failed to create folder: " << directory_path << std::endl;
        return false;
    }
    return true;
}

bool FileManager::CreateFile(std::ofstream &out_stream, std::string file_name) {
    out_stream.open(file_name, std::ios::app);
    if (!out_stream.is_open()) {
        std::cout << "Cannot create file: " << file_name << std::endl;
        return false;
    }
    return true;
}

} // namespace myslam