#pragma once
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <algorithm>
#include "general_task.h"
#include "cost_function.h"
#include <vector>
#include <queue>
#include "sql_client.h"

using namespace std;



class TaskManager{
public:
    TaskManager(SQLClient& sc,ros::NodeHandle& nh):_sc(sc),_nh(nh){
        _pc = _nh.serviceClient<nav_msgs::GetPlan>("/tb3_0/move_base/NavfnROS/make_plan"); 
        index = 0;
        LoadMapInfo();
    }

    void LoadMapInfo(){
        ROS_INFO("Loading door and charging station...");       
        auto v = _sc.QueryDoorId();
        _doors.swap(v);
        if(_doors.size()==0){
           ROS_INFO("No doors information");
           return;
        }else{
            random_shuffle(_doors.begin(),_doors.end());
        }
        _stations = _sc.QueryAvailableChargingStations();

        ROS_INFO_STREAM("Found %ld doors  and %ld charging station in database", _doors.size(),_stations.size());
    }

    void CreateNewTasks(int num){
        for(int i = 0; i < num ; i++){
            TaskInTable t;
            t.priority = 1;
            t.taskType = "GatherEnviromentInfo";
            t.goal.header.stamp = ros::Time::now() + ros::Duration(10*i);
            t.goal.header.frame_id = "map";
            t.targetId = _doors[index];
            t.taskId = _sc.InsertATaskAssignId(t);
            index++;
            if(index>=_doors.size()){
                index = 0;
            }
        }
    }


    TaskInTable CreateChargingTask(geometry_msgs::Pose robotPose){
        geometry_msgs::Pose rp = robotPose;
        ros::Time now = ros::Time::now();
        
        std::pair<int,double> best; // best charging station (id,distance) 
        best.second = 1000;  

        auto freeStations = _sc.QueryAvailableChargingStations();

        for(auto i : _stations){
            double dist = CalculatSmallTaskBatteryConsumption(rp,i.second);
            if(dist<best.second){
                best.first = i.first;
                best.second = dist;
            }
        }    
        TaskInTable bt;
        bt.taskType = "Charging";
        bt.priority = 5;
        bt.goal.header.stamp = now + ros::Duration(10); // create a charging task that start after 10s
        bt.goal.header.frame_id = "map";
        bt.goal.pose = _stations[best.first];   
        return bt;
    }

    // Convert dependend small tasks to large task
    vector<LargeTask> MakeLargeTasks(vector<TaskInTable>& sts){
        vector<LargeTask> lts;
        for(vector<TaskInTable>::iterator it = sts.begin(); it != sts.end(); ){
            if(it->dependency==0){ // Find task with no dependency
                LargeTask lt; // Create a large task
                lt.tasks.insert(make_pair(it->taskId,it->goal));
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
         ROS_INFO("Create %ld large task finished. Allocate remain %ld tasks",lts.size(),sts.size());
    
        while(!sts.empty()){
            for(vector<TaskInTable>::iterator it = sts.begin(); it != sts.end();){
                const int d = it->dependency; // Find dependency task
                vector<LargeTask>::iterator lit = find_if(lts.begin(),lts.end(),
                    [d](const LargeTask& l) ->bool {return l.tasks.count(d) > 0 ;}
                );
                if(lit == lts.end()){
                    ROS_INFO("Task %d dependence on unknown task",it->taskId);
                    _sc.UpdateTaskStatus(it->taskId,"Error"); // If a task is depend on unknown task, set it to error 
                }else{ 
                    lit->tasks.insert(make_pair(it->taskId,it->goal));
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

    // Calculate large task cost
    void CalculateLargeTasksCost(geometry_msgs::Pose robotPose,vector<LargeTask>& lts){
        ROS_INFO_STREAM("Large_task_id Battery WaitTime Open_possibility Priority   Cost");
        ROS_INFO("-----------------------------------------------------------------------------");
        ros::Time now = ros::Time::now();
        
        for(LargeTask t:lts){
            t.battery = CalculateLargeTaskBatteryConsumption(robotPose,t.tasks);
            t.waitingTime = t.tasks.begin()->second.header.stamp - now;
            CostCalculator::CalculateLargeTaskCost(t);
            ROS_INFO("%d        %.3f   %.3f   %.3f  %d  %3f",t.largeTaskId,t.battery,t.waitingTime.toSec(), t.openPossibility,t.priority,t.cost);
        }
    }

    // Sort large task with cost
    void SortLargeTasksWithCost(vector<LargeTask>& lts){
        sort(lts.begin(),lts.end(),
        [](const LargeTask&lt1,const LargeTask& lt2)->bool{
            return lt1.cost > lt2.cost;
        });
    }

    double CalculateLargeTaskBatteryConsumption(geometry_msgs::Pose robotPose,std::map<int,geometry_msgs::PoseStamped> tasks){
        double battery = 0.0;
        geometry_msgs::Pose start;
        map<int,geometry_msgs::PoseStamped>::iterator it = tasks.begin();
        battery += CalculatSmallTaskBatteryConsumption(robotPose,it->second.pose);
        start = it->second.pose;
        for( it++;it != tasks.end();it++){
          battery += CalculatSmallTaskBatteryConsumption(start,it->second.pose);
          start = it->second.pose;
        }
        return battery;
    }

    double CalculatSmallTaskBatteryConsumption(geometry_msgs::Pose start, geometry_msgs::Pose end){
            double distance = 0,angle = 0,batteryConsumption = 0;
        // for each task, request distance from move base plan server
            nav_msgs::GetPlan make_plan_srv;
            make_plan_srv.request.start.pose= start;
            make_plan_srv.request.start.header.frame_id = "map";
            make_plan_srv.request.goal.pose = end;
            make_plan_srv.request.goal.header.frame_id = "map";
            make_plan_srv.request.tolerance = 1;
	        
            // request plan with start point and end point
            if(!_pc.call(make_plan_srv)){
                ROS_INFO_STREAM("Failed to send request to make plan server");
                return -1;
            }
            // calculate distance
            std::vector<geometry_msgs::PoseStamped> &dists = make_plan_srv.response.plan.poses;

            if(dists.size()==0){
                ROS_DEBUG("Receive empty plan");
                return -1;
            };   
     
            for(size_t i = 1; i < dists.size();i++){
                distance = sqrt(pow((dists[i].pose.position.x - dists[i-1].pose.position.x),2) + 
                                    pow((dists[i].pose.position.y - dists[i-1].pose.position.y),2));
                                     
                angle = 2 * acos(dists[i].pose.orientation.w);
                batteryConsumption = batteryConsumption + 0.01 * distance + 0.001 * angle;
                // ROS_INFO("From (%.3f,%.3f) to (%.3f,%) linear variation = %.3f, angle variation = %.3f, battery consum = %.3f",
                        // dists[i-1].pose.position.x,dists[i-1].pose.position.y,dists[i].pose.position.x,dists[i].pose.position.y,distance,angle,batteryConsumption);
            }   
            return batteryConsumption;   
    }

    LargeTask SelectLargetask(geometry_msgs::Pose robotPose){
        ROS_INFO("Start query execute tasks...");
        vector<TaskInTable> sts;
        vector<LargeTask> lts;
        LargeTask lt;
        sts  = _sc.QueryRunableExecuteTasks();
        ROS_INFO("Found %ld execute tasks",sts.size());
        // FilterTask(v);
        if(sts.size() != 0 ){
            lts = MakeLargeTasks(sts);
            CalculateLargeTasksCost(robotPose,lts);
            ROS_INFO_STREAM("Calculate execute task cost finish");
            // FilterTask(v);
        }

         if(lts.size() == 0){ // after filter, if there is no execute task, gather inviroment
                while((sts = _sc.QueryRunableGatherEnviromentInfoTasks()).size() == 0){  // if no execute task, create some gather enviroment info task
                    CreateNewTasks(10);    
                }
                ROS_INFO("found %ld gather enviroment info tasks",sts.size());
                lts = MakeLargeTasks(sts);
                CalculateLargeTasksCost(robotPose,lts);
        }

        SortLargeTasksWithCost(lts);
        ROS_INFO_STREAM("Sort large task finish");
        lt = lts.back();
        return lt;
    }

    void SortTaskWithCost(std::vector<TaskInTable>& v){
         std::sort(v.begin(),v.end(),
        [](const TaskInTable& a, const TaskInTable&b)->bool
        {
                return a.cost >b.cost;
        });
    }

    // void FilterTask(std::vector<TaskInTable>& v){
    //     ros::Time now = ros::Time::now();
    //     ROS_INFO_STREAM("Filter Task  not expired and cost < "<<to_string(COST_LIMIT));
    //     for(vector<TaskInTable>::iterator it = v.begin(); it != v.end(); ){
    //         if(it->cost > COST_LIMIT || it ->goal.header.stamp < now ){
    //            ROS_INFO_STREAM("Delete task "<<it->taskId);
    //            it = v.erase(it);
    //         }else{
    //             it++;
    //         }
    //     }
    // }


    private:
    
    ros::ServiceClient _pc;
    SQLClient &_sc;
    ros::NodeHandle& _nh;
    vector<int> _doors;
    map<int,geometry_msgs::Pose> _stations;

    uint8_t index = 0;
};