#pragma once
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include "util.h"
#include <sstream>
using namespace std;

class Door{ 
    public:
    int doorId;
    geometry_msgs::Pose pose;
    double depOpenpossibility;
    ros::Time lastUpdate;
    bool isUsed;
    double cost;

    string getDoorInfo(){
        stringstream ss;
        ss << "Door "<< doorId <<"at ("<<pose.position.x<<", "<<pose.position.y<< 
        ") dependency open possibility" <<depOpenpossibility<<", last update "
        <<Util::time_str(lastUpdate)<< "is used "<<isUsed;

        return ss.str();
    }

    static void SortDoorsWithCost(vector<Door>& v){
        sort(v.begin(),v.end(),
        [](const Door& a, const Door& b)->bool
        {
            return a.cost > b.cost;
        });
    }

};