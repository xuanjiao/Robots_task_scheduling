#pragma once
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include "util.h"
#include <sstream>
using namespace std;

class Point{
public:
    int pointId;
    int doorId;
    geometry_msgs::PoseStamped goal;
    int depDoorId = 0;
};

class Door{ 
    public:
    int doorId;
    geometry_msgs::Pose pose;
    double depOpenpossibility;
    double openpossibility;
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


class ChargingStation{
public:
    int stationId;
    geometry_msgs::Pose pose;
    double remainingTime;
    double batteryLevel;
    double cost;

    static void SorChargingStationsWithCost(vector<ChargingStation>& v){
        sort(v.begin(),v.end(),
        [](const ChargingStation& a, const ChargingStation& b)->bool
        {
            return a.cost > b.cost;
        });
    }

    static void SortChargingStationsWithCost(vector<ChargingStation>& v){
        sort(v.begin(),v.end(),
        [](const ChargingStation& a, const ChargingStation& b)->bool
        {
            return a.cost > b.cost;
        });
    }
};