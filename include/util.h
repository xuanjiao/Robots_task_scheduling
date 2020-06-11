#pragma once

#include "ros/ros.h"
#include <string>
#include <sstream>
#include <geometry_msgs/PoseStamped.h>

class Util{
    public:
    static std::string pose_str(const geometry_msgs::Pose p){
        std::stringstream ss;
        ss.precision(3);
        ss << "("<<p.position.x <<", " <<p.position.y <<", "<< p.position.z<<")";
        return ss.str();
    }

    static std::string time_str(ros::Time time){
        const int output_size = 100;
        char output[output_size];
        std::string format = "%Y-%m-%d %H:%M:%S"; // time format
        std::time_t  raw_time = static_cast<time_t>(time.sec); // convert ros time to time_t
        struct tm* time_info = localtime(&raw_time);
        std::strftime(output,output_size,format.c_str(),time_info);
        return std::string(output);
    }

};