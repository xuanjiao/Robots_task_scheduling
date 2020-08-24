#pragma once
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include "util.h"

using namespace std;
class SmallTask{
public:
    virtual string getTaskInfo(){
        stringstream ss;
        ss<< taskType <<" : "<<Util::time_str(goal.header.stamp)<<
         " (" <<goal.pose.position.x<<","<<goal.pose.position.y<<")";
        return ss.str();
    }
    int taskId = 0;
    string taskType = "";
    int targetId = 0;
    int priority = 0;
    geometry_msgs::PoseStamped goal; // distination and timestamp
};

class SmallExecuteTask: public SmallTask{
public:
    int dependency = 0;
    double openPossibility = 0.0;
    double cost = 0.0;
};


class LargeTask{
public:
    int largeTaskId = 0;
    std::string taskType = "";
    std::map<int,SmallExecuteTask> smallTasks;
    double openPossibility = 0.0;
    double battery = 0.0;
    ros::Duration waitingTime;
    int priority = 0;
    double cost = 0.0;

    string getTaskInfo(){
        stringstream ss;
        ss<<"\n"<<taskType;
        for(auto it = smallTasks.begin(); it !=smallTasks.end(); it++){
           ss<<"\n["<<it->first<<"] "<< Util::time_str(it->second.goal.header.stamp)<<" "<< " (" <<it->second.goal.pose.position.x<<","<<it->second.goal.pose.position.y<<")";
        }
        return ss.str();
    }
};




