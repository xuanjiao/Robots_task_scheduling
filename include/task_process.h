#pragma once

#include "ros/ros.h"
#include "time_transfer.h"
#include <geometry_msgs/PoseStamped.h>

#define ROOM_NUM 4
#define TASK_NUM 50
#define DEFAULT_COST 1000
#define SIMULATION_DURATION_SEC 60 // 60s - 720min = 12 h

typedef struct {
    double path_lengh;
    int priority;
    int task_id;
    char room_id;
    geometry_msgs::PoseStamped goal; // distination and timestamp
}EnterRoomTask;


class TaskProcess{
public:
    TaskProcess(std::vector<std::pair<EnterRoomTask*,double>> &cost_vector):cost_vector(cost_vector){}
    // change returned tasks
    void change_returned_task(){
        
    }
    
    // create task
    void create_random_tasks(int num,ros::Time start_time,std::map<char,geometry_msgs::Pose> &room_map){
        ROS_INFO_STREAM("start create "<<num<<" tasks");
        ros::Time time;
        
        for(int i = 0; i < TASK_NUM ; i++)
        {              
            EnterRoomTask* task = new EnterRoomTask(); // Create a task
            task->goal.header.frame_id = "map";
            task->goal.header.stamp = start_time + ros::Duration(rand()%SIMULATION_DURATION_SEC);  // set task time a random time after start time
            task->priority = rand()%4 + 1;              // set random task priority 1-5
            task->room_id = rand()% ROOM_NUM + 'a';
            task->task_id = rand()% 100;
            task->goal.pose=room_map[task->room_id];                
                
            // add this task in tasks vector
            cost_vector.push_back(std::pair<EnterRoomTask*,double>(task,DEFAULT_COST));
               
            ROS_INFO_STREAM("Create new Task: time "<<TimeTransfer::convert_to_office_time_string(task->goal.header.stamp)<<
            " room "<<task->room_id<<" priority "<<task->priority <<" goal "<<task->goal);
        }  
    }


private:
    std::vector<std::pair<EnterRoomTask*,double>>& cost_vector;
};