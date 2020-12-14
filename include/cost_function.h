#pragma once
#include "task_type.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include "ros/ros.h"
#include "objects.h"

using namespace std;

typedef struct TaskWeight{
    double wt_btr = 5.0;
    double wt_wait = 1.0;
    double wt_psb = -5.0;
    double wt_pri = -5.0;
}TaskWeight;

typedef struct DoorWeight{
    double wt_update = 0.1; // large time since last update-> lower cost
    double wt_btr = 15;
    double wt_psb = 0.1;
}DoorWeight;

typedef struct ChargingWeight{
    double wt_remain   = 1;
    double wt_btr          = 1; // Battery cosumption
}ChargingWeight;

class CostCalculator{
    public:
    TaskWeight _tw;
    DoorWeight _dw;
    ChargingWeight _cw;
    CostCalculator(ros::NodeHandle &nh);
    void LoadTaskWeight(TaskWeight &tw);
    void LoadDoorWeight(DoorWeight &dw);
    void CalculateChargingStationCost(ChargingStation& cs, geometry_msgs::Pose robotPose);
    void CalculateLargeTasksCost(ros::Time now,LargeExecuteTask& t, geometry_msgs::Pose robotPose);
    void CalculateDoorCost(ros::Time now, Door& door, geometry_msgs::Pose robotPose);
    void CalculateComplexTrajectoryBatteryConsumption(geometry_msgs::Pose robotPose,LargeExecuteTask& lt);
    double CalculateSimpleBatteryConsumption(geometry_msgs::Pose start, geometry_msgs::Pose end);

private:
    ros::ServiceClient _pc;
    ros::NodeHandle _nh;
};
// llcf = {10,10/ntaks,0.1,-10,-1)}