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


    void HandleFailedTask(string taskType,const vector<int>& taskIds){
        if(taskType == "GatherEnviromentInfo"){
            _sc.UpdateTaskStatus(taskIds[0],"Error");
        }else if (taskType == "Charging"){
            _sc.UpdateTaskStatus(taskIds[0],"Canceled");
        }else if(taskType == "ExecuteTask"){
            // Change task status from Running to ToReRun, increase priority 3 and increase 60200s start time 
            _sc.UpdateFailedExecuteTask(taskIds);
        }else{
            ROS_INFO("Get a unknown task");
        }        
    }

    void HandleSucceededTask(const vector<int>& taskIds){
        for(auto it = taskIds.begin(); it != taskIds.end(); it++)
            ROS_INFO("Task Succedd. Update %d task status",_sc.UpdateTaskStatus(*it,"RanToCompletion"));
    }

    void AfterSendingTask(int taskId, int robotId){
        _sc.UpdateTaskStatus(taskId,"Running");
        _sc.UpdateTaskRobotId(taskId,robotId);
    }


    private:

    SQLClient& _sc;
    CostCalculator& _cc;
};