#pragma once
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include "util.h"

using namespace std;

class AbstractTask{
public:
    int taskId = 0;
    string taskType = "";
    int priority = 0;
    virtual string getTaskInfo() = 0;
};

class TaskRecord{
public:
    int taskId = 0;
    bool isOpen = false;
    int door_id;
};

class TaskResult{
public:
    vector<int> taskIds;
    bool isCompleted = false;
    string taskType;
    string description;
};

class SmallTask: public AbstractTask{
public:
    string getTaskInfo(){
        stringstream ss;
        ss<< taskType <<" : "<<Util::time_str(goal.header.stamp)<<
         " (" <<goal.pose.position.x<<","<<goal.pose.position.y<<")";
        return ss.str();
    }
    int targetId = 0;
    geometry_msgs::PoseStamped goal; // distination and timestamp
};


class SmallExecuteTask: public SmallTask{
public:
    int dependency = 0;
    double openPossibility = 0.0;
    double cost = 0.0;
};

class LargeExecuteTask: public AbstractTask{
public:
    map<int,SmallExecuteTask> smallTasks;
    double openPossibility = 0.0;
    double battery = 0.0;
    ros::Duration waitingTime;
    double cost = 0.0;
    static const int COST_LIMIT = 50; 

    string getTaskInfo(){
        stringstream ss;
        ss<<"\n"<<taskType;
        for(auto it = smallTasks.begin(); it !=smallTasks.end(); it++){
           ss<<"\n["<<it->first<<"] "<< Util::time_str(it->second.goal.header.stamp)<<" "<< " (" <<it->second.goal.pose.position.x<<","<<it->second.goal.pose.position.y<<")";
        }
        return ss.str();
    }

    // Sort large task with cost
    static void SortTasksWithCost(vector<LargeExecuteTask>& lts){
        sort(lts.begin(),lts.end(),
        [](const LargeExecuteTask&lt1,const LargeExecuteTask& lt2)->bool{
            return lt1.cost > lt2.cost;
        });
    }

    static void FilterTask(std::vector<LargeExecuteTask>& v){
        ROS_INFO_STREAM("Filter Task cost < "<<to_string(COST_LIMIT));
        for(vector<LargeExecuteTask>::iterator it = v.begin(); it != v.end(); ){
            if(it->cost > COST_LIMIT){
               ROS_INFO_STREAM("Delete large task "<<it->taskId);
               it = v.erase(it);
            }else{
                it++;
            }
        }

    }
    
    
    // Convert dependend small tasks to large task
    static vector<LargeExecuteTask> MakeLargeTasks(vector<SmallExecuteTask>& sts){
        vector<LargeExecuteTask> lts;
        for(vector<SmallExecuteTask>::iterator it = sts.begin(); it != sts.end(); ){
            if(it->dependency==0){ // Find task with no dependency
                LargeExecuteTask lt; // Create a large task
                lt.smallTasks.insert(make_pair(it->taskId,*it));
                lt.priority = it->priority;
                lt.openPossibility = it->openPossibility;
                lt.taskId = lts.size();
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
                vector<LargeExecuteTask>::iterator lit = find_if(lts.begin(),lts.end(),
                    [d](const LargeExecuteTask& l) ->bool {return l.smallTasks.count(d) > 0 ;}
                );
                if(lit == lts.end()){
                    ROS_INFO("Task %d dependence on unknown task",it->taskId);
                    // _sc.UpdateTaskStatus(it->taskId,"Error"); // If a task is depend on unknown task, set it to error 
                }else{ 
                    lit->smallTasks.insert(make_pair(it->taskId,*it));
                    lit->priority = it->priority;
                    lit->openPossibility = lit->openPossibility * it->openPossibility;
                    lit->taskId = lit - lts.begin();
                    ROS_INFO("Put task %d in large task %ld",it->taskId,lit - lts.begin());
                }
                it = sts.erase(it); // Erase it from task vector when finish calculating
            }
        }
        return lts;
    }
};




