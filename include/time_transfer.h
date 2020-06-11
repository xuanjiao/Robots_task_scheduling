#pragma once

#include "ros/ros.h"
#include <string>
#include <sstream>

#define START_TIME 1590980400 //2020.06.01 06:00:00

class TimeTransfer{
public:
    static ros::Time convert_to_office_time(ros::Time origin){
         uint32_t time = START_TIME + (origin.sec % 60) *720 + origin.sec / 60 * 86400; // 1s -12 min
         return ros::Time(time);
    }
    
    static std::string convert_to_office_time_string(ros::Time origin){
        const int output_size = 100;
        char output[output_size];
        std::string format = "%Y-%m-%d %H:%M:%S"; // time format
        std::time_t  raw_time = static_cast<time_t>(START_TIME + (origin.sec % 60) *720 + origin.sec / 60 * 86400); // convert ros time to time_t
        struct tm* time_info = localtime(&raw_time);
        std::strftime(output,output_size,format.c_str(),time_info);
        return std::string(output);
    }

};