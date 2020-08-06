#pragma once
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include "general_task.h"
#include <vector>
#include "sql_client.h"

#define COST_LIMIT 1000

using namespace std;

typedef struct cost_st{                                                                                                         
    double _distance = 0;
    double _sec_diff = 0;                                           
    int _priority = 0;
    double    _open_pos_st = 0;
    double _cost;
    cost_st(double distance,double sec_diff,double priority,double open_pos_st,double battery_level):
        _distance(distance),_sec_diff(sec_diff),_priority(priority),_open_pos_st(open_pos_st){
            _cost = 1.0 * _distance + 0.2 * _sec_diff + (-100) * _open_pos_st +(-10) * _priority  ;
        }
}CostFunction;


class TaskManager{
public:
    TaskManager(SQLClient& sc,ros::NodeHandle& nh):_sc(sc),_nh(nh){
        index = 0;
        _pc = _nh.serviceClient<nav_msgs::GetPlan>("/tb3_0/move_base/NavfnROS/make_plan"); 
    }

    void CreateNewTasks(int num){
        vector<int> doors = _sc.QueryDoorId();
        ROS_INFO_STREAM("Found " << doors.size() << " doors in database");
        if(doors.size()==0){
           ROS_INFO("No doors information");
           return;
        }
        for(int i = 0; i < num ; i++){
            TaskInTable t;
            t.priority = 1;
            t.taskType = "GatherEnviromentInfo";
            t.goal.header.stamp = ros::Time::now() + ros::Duration(100*i);
            t.goal.header.frame_id = "map";
            t.targetId = doors[index];
            t.taskId = _sc.InsertATaskAssignId(t);
            // ROS_INFO_STREAM("Created a task. \n target id = "<<t.targetId<< " task id ="<<t.taskId);
            index++;
            if(index>=doors.size()){
                index = 0;
            }
        }

    }

        
    TaskInTable GetBestTask(vector<TaskInTable> & v,geometry_msgs::Pose robot_pose,ros::Time t,double battery){
        ROS_INFO_STREAM("Task_id  Task_type Target_id Priority Open_pos Distance Sec_diff Cost");
        ROS_INFO("-----------------------------------------------------------------------------");
        for(auto &i :v){
               CostFunction cf(
                   calculateBatteryConsumption(i.goal.pose,robot_pose),
                   i.goal.header.stamp.sec - t.sec,
                   i.priority,
                   i.openPossibility,
                   battery
                );
                i.cost = cf._cost;

                ROS_INFO_STREAM(
                    setw(3) <<i.taskId <<" "<<setw(5)<<i.taskType <<setw(4) << i.targetId << setw(2) << i.priority << setprecision(3)<< setw(4) << 
                    i.openPossibility << " " << setprecision(3)<< setw(5) << cf._distance << " "<<fixed<<setprecision(3) << setw(10) <<cf._sec_diff<< " " <<fixed << setprecision(3) << setw(6) <<i.cost 
                );
        }

        std::sort(v.begin(),v.end(),
        [](const TaskInTable& a, const TaskInTable&b)->bool
        {
                return a.cost >b.cost;
        });

        for(vector<TaskInTable>::iterator it = v.begin(); it != v.end(); it++){
            ROS_INFO_STREAM("Task with cost < "<<to_string(COST_LIMIT));
            if(it->cost > COST_LIMIT){
               ROS_INFO_STREAM(it->taskId<<" "<<it->cost<<" (cost > " << to_string(COST_LIMIT) << " delete)");
               v.erase(it);
            }else{
                ROS_INFO_STREAM(it->taskId<<" "<<it->cost);
            }
        }
        return v.back();
    }
    
    vector<TaskInTable> calculateCostofTasks(vector<TaskInTable> &tasks, geometry_msgs::Pose robotPose){
        ros::Time now = ros::Time::now();
        vector<TaskInTable> tasksWithCost;
        double batteryConsumption = 0,openPossibility = 0;
        int priority = 0,waitTime = 0;
       
        ROS_INFO_STREAM("id type     Target_id Priority Open_pos Battery WaitTime Cost");
        ROS_INFO("-----------------------------------------------------------------------------");
        for(vector<TaskInTable>::iterator it = tasks.begin(); it != tasks.end(); ){
            if(it->dependency==0){ // Find task with no dependency
                batteryConsumption = calculateBatteryConsumption(robotPose,it->goal.pose);
                waitTime = it->goal.header.stamp.sec - now.sec;
                openPossibility = it->openPossibility;
                priority = it->priority;
                it->cost =    1.0 * batteryConsumption + 0.2 * waitTime + (-100) * openPossibility + (-10) * priority; 
                tasksWithCost.push_back(*it);   
                ROS_INFO("%d  %s %d   %d  %.2f  %.2f  %d  %.2f",it->taskId,it->taskType.c_str(),it->targetId,priority,openPossibility,batteryConsumption,waitTime,it->cost);

                it = tasks.erase(it); // Erase it from task vector when finish calculating            
            }else{
                it++;
            }
        }

       while(!tasks.empty()){
            for(vector<TaskInTable>::iterator it = tasks.begin(); it != tasks.end();){
                const int d = it->dependency;
                vector<TaskInTable>::iterator dependencyTaskIt = find_if(tasksWithCost.begin(),tasksWithCost.end(),
                    [d](const TaskInTable& t) ->bool {return t.taskId == d;}
                );
                if(dependencyTaskIt == tasksWithCost.end()){
                    ROS_INFO("Task %d dependence on unknown task",it->taskId);
                    _sc.UpdateTaskStatus(it->taskId,"Error"); // If a task is depend on unknown task, set it to error 
                }else{ 
                    // If task is depend on another task
                    batteryConsumption = calculateBatteryConsumption(dependencyTaskIt->goal.pose,it->goal.pose);
                    waitTime = it->goal.header.stamp.sec - now.sec;
                    openPossibility = it->openPossibility * dependencyTaskIt->openPossibility;
                    priority = it->priority;
                    it->cost =  dependencyTaskIt->cost +  1.0 * batteryConsumption + 0.2 * waitTime + (-100) * openPossibility + (-10) * priority; 
                    tasksWithCost.push_back(*it);

                     ROS_INFO("%d  %s %d   %d  %.2f  %.2f  %d  %.2f",it->taskId,it->taskType.c_str(),it->targetId,priority,openPossibility,batteryConsumption,waitTime,it->cost);

                }
                    it = tasks.erase(it); // Erase it from task vector when finish calculating
            }
       }

       return tasksWithCost;
    }



    double calculateBatteryConsumption(geometry_msgs::Pose end, geometry_msgs::Pose start){
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
                // ROS_INFO("From (%.2f,%.2f) to (%.2f,%) linear variation = %.2f, angle variation = %.2f, battery consum = %.2f",
                        // dists[i-1].pose.position.x,dists[i-1].pose.position.y,dists[i].pose.position.x,dists[i].pose.position.y,distance,angle,batteryConsumption);
            }   
            return batteryConsumption;   
    }


    TaskInTable SelectBestTask(geometry_msgs::Pose robotPose){
            std::vector<TaskInTable> v;

            v = _sc.QueryRunableExecuteTasks();  // find if there are execute task    
            ROS_INFO_STREAM("found "<<v.size()<<" execute tasks");
            v = calculateCostofTasks(v,robotPose); // calculate cost and delect problem task
            FilterTask(v);

            if(v.size() == 0){ // after filter, if there is no execute task, gather inviroment
                while((v = _sc.QueryRunableGatherEnviromentInfoTasks()).size() == 0){  // if no execute task, create some gather enviroment info task
                    CreateNewTasks(10);    
                    ros::Duration(2).sleep();            
                }
                ROS_INFO_STREAM("found "<<v.size()<<"gather enviroment info tasks");
                v = calculateCostofTasks(v,robotPose); // calculate cost
                FilterTask(v);
            }
            
            SortTaskWithCost(v);
            TaskInTable bt = v.back();
            ROS_INFO_STREAM("Best task id = "<<bt.taskId<<" ,cost = "<< fixed << setprecision(3) << setw(6)<< bt.cost);
            return bt;
        
    }

    void SortTaskWithCost(std::vector<TaskInTable>& v){
         std::sort(v.begin(),v.end(),
        [](const TaskInTable& a, const TaskInTable&b)->bool
        {
                return a.cost >b.cost;
        });
    }

    void FilterTask(std::vector<TaskInTable>& v){
        ros::Time now = ros::Time::now();
        ROS_INFO_STREAM("Filter Task  not expired and cost < "<<to_string(COST_LIMIT));
        for(vector<TaskInTable>::iterator it = v.begin(); it != v.end(); ){
            if(it->cost > COST_LIMIT || it ->goal.header.stamp < now ){
               ROS_INFO_STREAM("Delete task "<<it->taskId);
               it = v.erase(it);
            }else{
                it++;
            }
        }
    }


    private:
    ros::ServiceClient _pc;
    SQLClient &_sc;
    ros::NodeHandle& _nh;
    // vector<int> doors;
    uint8_t index;
   
};