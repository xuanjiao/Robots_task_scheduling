#pragma once

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <algorithm>
#include "task_type.h"
#include "cost_function.h"
#include <vector>
#include <queue>
#include <sstream>
#include <set>
#include "sql_client.h"
#include "room_map.h"

using namespace std;


class TaskManager{
public:
    static const int TASK_DELAY = 10;
    static const int ENV_EXP_TIME = 1200;
    ros::Timer _exp_timer;
    ros::NodeHandle& _nh;

    TaskManager(ros::NodeHandle& nh,SQLClient& sc, CostCalculator& cc);
    void FinishEnviromentExperiment(const ros::TimerEvent& event);
    SmallTask GetAChargingTask(int robotId);
    LargeExecuteTask SelectExecutetask(int robotId, geometry_msgs::Pose robotPose);
    SmallTask CreateBestEnviromentTask(geometry_msgs::Pose robotPose);
    SmallTask CreateBestChargingTask(geometry_msgs::Pose robotPose);
    void HandleTaskFeedback(TaskFeedback& fb);
    void HandleTaskResult(TaskResult result);
    void AfterSendingTask(int taskId, int robotId);
    RltDoors CalculatePossibilityProduct(Door& d,geometry_msgs::Pose robotPose);
    RltDoors CalculatePossibilityProduct(LargeExecuteTask& t);
private:

    int _exp_id;
    SQLClient& _sc;
    CostCalculator& _cc;
};