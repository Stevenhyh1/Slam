#ifndef CONFIG_H
#define CONFIG_H

#include "utils.h"
#include "camera.h"
#include "mappoint.h"
#include "frame.h"
#include "map.h"

namespace myvo
{
    class Config
    {
        private:
            static std::shared_ptr<Config> config_;
            cv::FileStorage file_;
            Config () {}
        public:
            ~Config()

            static void setParameterFile( const std::string & filename);
            template<typename T>
            static T get(const std::string & key) {
                return T(Config::config_ -> file_[key]);
            }

    };

}

#endif