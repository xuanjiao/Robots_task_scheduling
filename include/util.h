#pragma once

#include "ros/ros.h"
#include <string>
#include <sstream>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
class Util{
    public:
    static string pose_str(const geometry_msgs::Pose p){
        stringstream ss;
        ss.precision(3);
        ss << "("<<p.position.x <<", " <<p.position.y <<", "<< p.position.z<<")";
        return ss.str();
    }

    static string time_str(ros::Time time){
        const int output_size = 100;
        char output[output_size];
        string format = "%Y-%m-%d %H:%M:%S"; // time format
        time_t  raw_time = static_cast<time_t>(time.sec); // convert ros time to time_t
        struct tm* time_info = localtime(&raw_time);
        strftime(output,output_size,format.c_str(),time_info);
        return string(output);
    }

    static ros::Time str_ros_time(string s){
        struct tm t;
        ros::Time rt;
        strptime(s.c_str(),"%Y-%m-%d %H:%M:%S",&t);
        rt.sec = mktime(&t);
        return rt;
    }

};