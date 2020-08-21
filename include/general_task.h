#pragma once
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include "util.h"

using namespace std;
class TaskInTable{
public:
    int taskId = 0;
    int dependency = 0;
    std::string taskType = "";
    int targetId = 0;
    double openPossibility = 0.0;
    int priority = 0;
    geometry_msgs::PoseStamped goal; // distination and timestamp
    double cost = 0.0;

    string getTaskInfo(){
        stringstream ss;
        ss<<"\n"<< taskType <<" : "<<Util::time_str(goal.header.stamp)<<
         " (" <<goal.pose.position.x<<","<<goal.pose.position.y<<")";
        return ss.str();
    }

};

class LargeTask{
public:
    int largeTaskId = 0;
    std::string taskType = "";
    std::map<int,geometry_msgs::PoseStamped> tasks;
    double openPossibility = 0.0;
    double battery = 0.0;
    ros::Duration waitingTime;
    int priority = 0;
    double cost = 0.0;

    string getTaskInfo(){
        stringstream ss;
        ss<<"\n"<<taskType;
        for(auto it = tasks.begin(); it !=tasks.end(); it++){
           ss<<"\n["<<it->first<<"] "<< Util::time_str(it->second.header.stamp)<<" "<< " (" <<it->second.pose.position.x<<","<<it->second.pose.position.y<<")";
        }
        return ss.str();
    }
};




