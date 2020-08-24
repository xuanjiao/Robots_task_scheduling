#pragma once
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <algorithm>
#include "general_task.h"
#include "cost_function.h"
#include <vector>
#include <queue>
#include <set>
#include "sql_client.h"

using namespace std;

#define COST_LIMIT 50

class TaskManager{
public:
    TaskManager(SQLClient& sc, CostCalculator& cc):_sc(sc),_cc(cc){
        
        // index = 0;
        // LoadMapInfo();
    }

    // void LoadMapInfo(){
    //     ROS_INFO("Loading door and charging station...");       
    //     auto v = _sc.QueryDoorId();
    //     _doors.swap(v);
    //     if(_doors.size()==0){
    //        ROS_INFO("No doors information");
    //        return;
    //     }else{
    //         random_shuffle(_doors.begin(),_doors.end());
    //     }
    //     // _stations = _sc.QueryAvailableChargingStations();
    //     // ROS_INFO("Found %ld doors  and %ld charging station in database", _doors.size(),_stations.size());
    // }

    // void CreateNewTasks(int num){
    //     for(int i = 0; i < num ; i++){
    //         SmallExecuteTask t;
    //         t.priority = 1;
    //         t.taskType = "GatherEnviromentInfo";
    //         t.goal.header.stamp = ros::Time::now() + ros::Duration(10*i);
    //         t.goal.header.frame_id = "map";
    //         t.targetId = _doors[index];
    //         t.taskId = _sc.InsertATaskAssignId(t);
    //         index++;
    //         if(index>=_doors.size()){
    //             index = 0;
    //         }
    //     }
    // }


    // SmallExecuteTask CreateChargingTask(geometry_msgs::Pose robotPose){
    //     geometry_msgs::Pose rp = robotPose;
    //     ros::Time now = ros::Time::now();
        
    //     std::pair<int,double> best; // best charging station (id,distance) 
    //     best.second = 1000;  

    //     auto freeStations = _sc.QueryAvailableChargingStations();

    //     for(auto i : _stations){
    //         double dist = CalculateSimpleBatteryConsumption(rp,i.second);
    //         if(dist<best.second){
    //             best.first = i.first;
    //             best.second = dist;
    //         }
    //     }    
    //     SmallExecuteTask bt;
    //     bt.taskType = "Charging";
    //     bt.priority = 5;
    //     bt.goal.header.stamp = now + ros::Duration(10); // create a charging task that start after 10s
    //     bt.goal.header.frame_id = "map";
    //     bt.goal.pose = _stations[best.first];   
    //     return bt;
    // }

    // Convert dependend small tasks to large task
    vector<LargeTask> MakeLargeTasks(vector<SmallExecuteTask>& sts){
        vector<LargeTask> lts;
        for(vector<SmallExecuteTask>::iterator it = sts.begin(); it != sts.end(); ){
            if(it->dependency==0){ // Find task with no dependency
                LargeTask lt; // Create a large task
                lt.smallTasks.insert(make_pair(it->taskId,*it));
                lt.priority = it->priority;
                lt.openPossibility = it->openPossibility;
                lt.largeTaskId = lts.size();
                lt.taskType = it->taskType;
                lts.push_back(lt);
                it = sts.erase(it);
            }else{
                it++;
            }
        }
         ROS_INFO("Create %ld large task finished. Allocate remain %ld smallTasks",lts.size(),sts.size());
    
        while(!sts.empty()){
            for(vector<SmallExecuteTask>::iterator it = sts.begin(); it != sts.end();){
                const int d = it->dependency; // Find dependency task
                vector<LargeTask>::iterator lit = find_if(lts.begin(),lts.end(),
                    [d](const LargeTask& l) ->bool {return l.smallTasks.count(d) > 0 ;}
                );
                if(lit == lts.end()){
                    ROS_INFO("Task %d dependence on unknown task",it->taskId);
                    _sc.UpdateTaskStatus(it->taskId,"Error"); // If a task is depend on unknown task, set it to error 
                }else{ 
                    lit->smallTasks.insert(make_pair(it->taskId,*it));
                    lit->priority = it->priority;
                    lit->openPossibility = lit->openPossibility * it->openPossibility;
                    lit->largeTaskId = lit - lts.begin();
                    ROS_INFO("Put task %d in large task %ld",it->taskId,lit - lts.begin());
                }
                it = sts.erase(it); // Erase it from task vector when finish calculating
            }
        }
        return lts;
    }



    // Sort large task with cost
    void SortLargeTasksWithCost(vector<LargeTask>& lts){
        sort(lts.begin(),lts.end(),
        [](const LargeTask&lt1,const LargeTask& lt2)->bool{
            return lt1.cost > lt2.cost;
        });
    }

    LargeTask SelectExecutetask(geometry_msgs::Pose robotPose){
        ROS_INFO("Start query execute tasks...");
        vector<SmallExecuteTask> sts;
        vector<LargeTask> lts;
        LargeTask lt;
        ros::Time now = ros::Time::now();
        sts  = _sc.QueryRunableExecuteTasks();
        ROS_INFO("Found %ld execute tasks",sts.size());
        // FilterTask(v);
        if(sts.size() != 0 ){
            lts = MakeLargeTasks(sts);
            ROS_INFO_STREAM("Large_task_id Battery WaitTime Open_possibility Priority   Cost");
            ROS_INFO("-----------------------------------------------------------------------------");
            for(LargeTask& t:lts){
                _cc.CalculateLargeTasksCost(now,t,robotPose);
                ROS_INFO_STREAM("Calculate execute task cost finish");
            } 
            FilterTask(lts); // remove task exceed cost limit
        }
        if(lts.size() != 0){ // after filter, if there is no execute task, gather inviroment
            SortLargeTasksWithCost(lts);
            lt = lts.back();
            ROS_INFO("Best execute task is %d",lt.largeTaskId);
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
        SortDoorsWithCost(doors);

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


    void SortDoorsWithCost(vector<Door>& v){
        sort(v.begin(),v.end(),
        [](const Door& a, const Door& b)->bool
        {
            return a.cost > b.cost;
        });
    }

    void SortTaskWithCost(std::vector<SmallExecuteTask>& v){
         std::sort(v.begin(),v.end(),
        [](const SmallExecuteTask& a, const SmallExecuteTask&b)->bool
        {
                return a.cost >b.cost;
        });
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

    void FilterTask(std::vector<SmallExecuteTask>& v){
        ROS_INFO_STREAM("Filter Task cost < "<<to_string(COST_LIMIT));
        for(vector<SmallExecuteTask>::iterator it = v.begin(); it != v.end(); ){
            if(it->cost > COST_LIMIT){
               ROS_INFO_STREAM("Delete small task "<<it->taskId);
               it = v.erase(it);
            }else{
                it++;
            }
        }
    }

    void FilterTask(std::vector<LargeTask>& v){
        ROS_INFO_STREAM("Filter Task cost < "<<to_string(COST_LIMIT));
        for(vector<LargeTask>::iterator it = v.begin(); it != v.end(); ){
            if(it->cost > COST_LIMIT){
               ROS_INFO_STREAM("Delete large task "<<it->largeTaskId);
               it = v.erase(it);
            }else{
                it++;
            }
        }
    }

    private:

    SQLClient& _sc;
    CostCalculator& _cc;
    
    
    // ros::NodeHandle& _nh;
    // vector<int> _doors;
    // map<int,geometry_msgs::Pose> _stations;

    // uint8_t index = 0;
};