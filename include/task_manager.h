#pragma once

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <algorithm>
#include "task_type.h"
#include "cost_function.h"
#include <vector>
#include <queue>
#include <set>
#include "sql_client.h"

using namespace std;


class TaskManager{
public:
    TaskManager(SQLClient& sc, CostCalculator& cc):_sc(sc),_cc(cc){

    }




    LargeExecuteTask SelectExecutetask(geometry_msgs::Pose robotPose){
        ROS_INFO("Start query execute tasks...");
        vector<SmallExecuteTask> sts;
        vector<LargeExecuteTask> lts;
        LargeExecuteTask lt;
        ros::Time now = ros::Time::now();
        sts  = _sc.QueryRunableExecuteTasks();
        ROS_INFO("Found %ld execute tasks",sts.size());
        // FilterTask(v);
        if(sts.size() != 0 ){
            lts = LargeExecuteTask::MakeLargeTasks(sts);
            ROS_INFO_STREAM("Large_task_id Battery WaitTime Open_possibility Priority   Cost");
            ROS_INFO("-----------------------------------------------------------------------------");
            for(LargeExecuteTask& t:lts){
                _cc.CalculateLargeTasksCost(now,t,robotPose);
                ROS_INFO_STREAM("Calculate execute task cost finish");
            } 
            LargeExecuteTask::FilterTask(lts); // remove task exceed cost limit
        }
        if(lts.size() != 0){ // after filter, if there is no execute task, gather inviroment
            LargeExecuteTask::SortTasksWithCost(lts);
            lt = lts.back();
            ROS_INFO("Best execute task is %d",lt.taskId);
        }
        return lt;
    }

    SmallTask CreateBestEnviromentTask(geometry_msgs::Pose robotPose){
        SmallTask st;
        ros::Time now = ros::Time::now();
        auto doors = _sc.QueryDoorInfo();
        ROS_INFO("Found %ld doors",doors.size());
        if(doors.empty()){
            ROS_INFO("No door data in database");
            exit(1);
        }
        ROS_INFO_STREAM("Id BatteryComsume TimeSinceLastUpdate Openpossibility IsUsed Cost");
        ROS_INFO("-----------------------------------------------------------------------------");
        for(Door& door : doors){
                // this door is not exploring by other doors;
                _cc.CalculateDoorCost(now,door,robotPose);     
            //ROS_INFO("calculate from  (%s) door %d to (%s)",Util::pose_str(robotPose).c_str(), door.doorId, Util::pose_str(door.pose).c_str());
        }
        Door::SortDoorsWithCost(doors);

        ROS_INFO("Best door is %d",doors.back().doorId);
        st.targetId = doors.back().doorId;
        st.goal.pose = doors.back().pose;
        st.goal.header.stamp = now + ros::Duration(10);
        st.goal.header.frame_id = "map";
        st.taskType = "GatherEnviromentInfo";
        st.priority = 1;
        st.taskId =  _sc.InsertATaskAssignId(st);
        return st;
    }

    SmallTask CreateBestChargingTask(geometry_msgs::Pose robotPose){
        SmallTask st;
        ros::Time now = ros::Time::now();
        auto css = _sc.QueryChargingStationInfo();
        if(css.empty()){
            ROS_INFO("No door data in database");
            exit(1);
        }
        ROS_INFO_STREAM("Id battery level remaining time Cost");
        ROS_INFO("-----------------------------------------------------------------------------");
        for(ChargingStation& cs : css){
                _cc.CalculateChargingStationCost(cs,robotPose);     
        }
        ChargingStation::SorChargingStationsWithCost(css);

        ROS_INFO("Best charging station is %d",css.back().stationId);
        st.targetId = css.back().stationId;
        st.goal.pose = css.back().pose;
        st.goal.header.stamp = now + ros::Duration(10);
        st.goal.header.frame_id = "map";
        st.taskType = "Charging";
        st.priority = 5;
        st.taskId =  _sc.InsertATaskAssignId(st);
        return st;
    }


    void HandleTaskResult(TaskResult& result){
        if(result.isCompleted){
            for(auto it = result.taskIds.begin(); it != result.taskIds.end(); it++){
                int ret1 = _sc.UpdateTaskStatus(*it,"RanToCompletion");
                int ret2 = _sc.UpdateTaskDescription(*it,result.description);
                ROS_INFO("Task Succedd. Update task status %d description %d",ret1,ret2);
            }
        }else{
            if(result.taskType == "GatherEnviromentInfo"){
                _sc.UpdateTaskStatus(result.taskIds[0],"Error");
            }else if (result.taskType == "Charging"){
                _sc.UpdateTaskStatus(result.taskIds[0],"Canceled");
            }else if(result.taskType == "ExecuteTask"){
                // Change task status from Running to ToReRun, increase priority 3 and increase 60200s start time 
                _sc.UpdateFailedExecuteTask(result.taskIds);
            }else{
                ROS_INFO("Get a unknown task");
            } 
        }
    }

    void AfterSendingTask(int taskId, int robotId){
        _sc.UpdateTaskStatus(taskId,"Running");
        _sc.UpdateTaskRobotId(taskId,robotId);
    }


    private:

    SQLClient& _sc;
    CostCalculator& _cc;
};