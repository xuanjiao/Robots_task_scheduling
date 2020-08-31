#pragma once
#include <vector>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
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