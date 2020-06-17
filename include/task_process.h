#pragma once

#include "ros/ros.h"
#include "time_transfer.h"
#include <geometry_msgs/PoseStamped.h>

#define ROOM_NUM 3
#define TASK_NUM 20
#define DEFAULT_COST 1000
#define SIMULATION_DURATION_SEC 100 // 100s 
#define DELAY_RETURNED_TASK_SEC 5   // 5s - 60min  = 1h  
#define INCREASE_RETURNED_TASK_PRIORITY    3
#define TASK_INTERVAL_SEC 30

typedef struct {
    double path_lengh;
    int priority;
    int task_id;
    char room_id;
    geometry_msgs::PoseStamped goal; // distination and timestamp
}EnterRoomTask;


class TaskProcess{
public:
    TaskProcess(
        std::vector<std::pair<EnterRoomTask*,double>> &cost_vector,
        std::map<int,EnterRoomTask*> &doing_task):
        cost_vector(cost_vector),
        doing_task(doing_task){}
    

    // change returned task and put it back to pool
    void change_returned_task(int id){
        EnterRoomTask* task = doing_task[id];
        task->goal.header.stamp = ros::Time::now() + ros::Duration(DELAY_RETURNED_TASK_SEC);
        task->priority += INCREASE_RETURNED_TASK_PRIORITY;
        if(task->priority>5){
            task->priority = 5;
        }
        cost_vector.push_back(std::pair<EnterRoomTask*,double>(task,DEFAULT_COST));
    }

    void delete_finished_task(int id){
        if(doing_task.empty()){
            ROS_INFO_STREAM("No runing task. Nothing to delete");
        }else if(doing_task[id]!=NULL){
            delete doing_task[id];
            doing_task.erase(id);
            ROS_INFO_STREAM("Task "<<id<< " finished. Erase it");
        }else{
            ROS_INFO_STREAM("Unknown task.Nothing to delete");
        }
    }
    // // create task
    // void create_enter_room_tasks(int num,ros::Time start_time){
    //     for(inti = 0; i < num; i++){
    //         sql_client.insert_new_task("EnterRoomTask",
    //             TimeTransfer::convert_to_office_time_string(start_time + ros::Duration(TASK_INTERVAL_SEC*i)));
    //     }
    // }

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
    std::map<int,EnterRoomTask*>& doing_task;
};