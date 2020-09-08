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

using namespace std;

class TaskManager{
public:


    TaskManager(SQLClient& sc, CostCalculator& cc):_sc(sc),_cc(cc){
        
       // dm[Key(0,3)] = {1}; 
       // dm[Key(0,3)] = {1}; 

    };




    LargeExecuteTask SelectExecutetask(geometry_msgs::Pose robotPose){
        ROS_INFO("Start query execute tasks...");
        vector<SmallExecuteTask> sts;
        vector<LargeExecuteTask> lts;
        LargeExecuteTask lt;
        ros::Time now = ros::Time::now();
        auto w = _sc.QueryTaskWeight();
        _cc.LoadWeight(w);
        sts  = _sc.QueryRunableExecuteTasks();
        ROS_INFO("Found %ld execute tasks",sts.size());
        // FilterTask(v);
        if(sts.size() != 0 ){
            lts = LargeExecuteTask::MakeLargeTasks(sts);
            ROS_INFO_STREAM("Large_task_id Battery WaitTime Open_possibility Priority   Cost");
            ROS_INFO("-----------------------------------------------------------------------------");
            for(LargeExecuteTask& t:lts){
                CalculateLargetaskOpenpossibility(t);     
                _cc.CalculateLargeTasksCost(now,t,robotPose);
                // ROS_INFO_STREAM("Calculate execute task cost finish");
            } 
            // LargeExecuteTask::FilterTask(lts); // remove task exceed cost limit
        }

        stringstream ss;
        
        if(lts.size() != 0){ // after filter, if there is no execute task, gather inviroment
          //  ss << "After filter, there are tasks: ";
          //  for(LargeExecuteTask& t:lts){
           //     ss << t.taskId <<" ";
          //  } 
            
            LargeExecuteTask::SortTasksWithCost(lts);
            lt = lts.back();
            ss << "the best is "<< lt.taskId;
        }else{
           // ss << "After filter, there are no execute tasks.";
        }
        ROS_INFO_STREAM(ss.str());
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
        ROS_INFO_STREAM("Id remaining time battery level  Cost");
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

    void HandleTaskFeedback(TaskFeedback& fb){
        int r = _sc.InsertDoorStatusRecord(fb.doorId,fb.measureTime,fb.doorStatus); 
        int u = _sc.UpdateOpenPossibilities(fb.doorId,fb.measureTime);
        ROS_INFO("Insert %d record, update %d rows in possibility table",r,u);
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
                _sc.UpdateTaskDescription(result.taskIds[0],result.description);
            }else if (result.taskType == "Charging"){
                _sc.UpdateTaskStatus(result.taskIds[0],"Canceled");
                 _sc.UpdateTaskDescription(result.taskIds[0],result.description);
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

    void CalculateLargetaskOpenpossibility(LargeExecuteTask& t){
        set<int> doors;
        stringstream ss;
        int id = 0;
        ss << "Relative doors list: ";
        for( auto sit =t.smallTasks.begin(); sit !=t.smallTasks.end(); sit++){          
            // find all doors and dependency doors 
            id = sit->second.point.doorId;
            doors.insert(id);
            ss << " "<<id;
            id = sit->second.point.depDoorId;
            doors.insert(id);
            ss << " "<<id;
            id = 0;            
        }
        
        doors.erase(0); // ignore 0
        vector<double> ops =  _sc.QueryRelativeDoorOpenPossibility(doors,t.waitingTime);
        
        t.openPossibility = 1;
        for(auto op : ops){
            t.openPossibility *= op;
        }
        // ROS_INFO_STREAM("Task related door: "<<ss.str()<<" multiply open possibility "<<t.openPossibility);
    }
    private:

    SQLClient& _sc;
    CostCalculator& _cc;
};