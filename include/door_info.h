#pragma once
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include "util.h"

class Door{ 
    public:
    int doorId;
    geometry_msgs::Pose pose;
    double depOpenpossibility;
    ros::Time lastUpdate;
    double cost;

    void printDoorInfo(){
        ROS_INFO("Door %d at (%f,%f) dependency open possibility %f, last update %s",
            doorId,pose.position.x,pose.position.y,depOpenpossibility,Util::time_str(lastUpdate).c_str());
    }

};