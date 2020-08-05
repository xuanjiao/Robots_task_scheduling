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
    double _battery_level = 0;
    double _cost;
    cost_st(double distance,double sec_diff,double priority,double open_pos_st,double battery_level):
        _distance(distance),_sec_diff(sec_diff),_priority(priority),_open_pos_st(open_pos_st),_battery_level(battery_level){
            _cost = 1.0 * _distance + 0.2 * _sec_diff + (-100) * _open_pos_st +(-10) * _priority  + (-1.0) * _battery_level;
        }
}CostFunction;


class TaskManager{
public:
    TaskManager(SQLClient& sc,ros::NodeHandle& nh):_sc(sc),_nh(nh){
        index = 0;
        _pc = _nh.serviceClient<nav_msgs::GetPlan>("/tb3_0/move_base/NavfnROS/make_plan"); 
        doors = _sc.QueryDoorId();
        if(doors.size()==0){
           ROS_INFO("No doors information");
           return;
        };
    }

    void CreateNewTasks(int num){
        int taskId;
        ROS_INFO_STREAM("Found " << doors.size() << " doors in database");
        if(doors.size()==0){
           ROS_INFO("No doors information");
           return;
        };
        for(int i = 0; i < num ; i++){
            Task t;
            t.priority = 1;
            t.taskType = "GatherEnviromentInfo";
            t.goal.header.stamp = ros::Time::now() + ros::Duration(100*i);
            t.goal.header.frame_id = "map";
            t.targetId = doors[index];
            taskId = _sc.InsertATaskAssignId(t);
            ROS_INFO_STREAM("Created a task. \n"<<t.goal<<" target id = "<<t.targetId<< " task id ="<<taskId);
            index++;
            if(index>=doors.size()){
                index = 0;
            }
        }

    }

        
    Task GetBestTask(vector<Task> & v,geometry_msgs::Pose robot_pose,ros::Time t,double battery){
        ROS_INFO_STREAM("Task_id  Task_type Target_id Priority Open_pos Distance Sec_diff Cost");
        ROS_INFO("-----------------------------------------------------------------------------");
        for(auto &i :v){
               CostFunction cf(
                   calculate_distance(i.goal.pose,robot_pose),
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
        [](const Task& a, const Task&b)->bool
        {
                return a.cost >b.cost;
        });

        for(vector<Task>::iterator it = v.begin(); it != v.end(); it++){
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


    double calculate_distance(geometry_msgs::Pose target_pose, geometry_msgs::Pose robot_pose){
            double distance = 0;
        // for each task, request distance from move base plan server
            nav_msgs::GetPlan make_plan_srv;
            make_plan_srv.request.start.pose= robot_pose;
            make_plan_srv.request.start.header.frame_id = "map";
            make_plan_srv.request.goal.pose = target_pose;
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
                distance += sqrt(pow((dists[i].pose.position.x - dists[i-1].pose.position.x),2) + 
                                    pow((dists[i].pose.position.y - dists[i-1].pose.position.y),2));
            }   

            return distance;    
    }

    private:
    ros::ServiceClient _pc;
    SQLClient &_sc;
    ros::NodeHandle& _nh;
    vector<int> doors;
    int index;
   
};