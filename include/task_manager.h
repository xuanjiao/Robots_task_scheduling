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
            ROS_INFO_STREAM("Found " << _doors.size() << " doors in database");
            random_shuffle(_doors.begin(),_doors.end());
        }

        _stations = _sc.QueryAvailableChargingStations();
        for(size_t i = 0; i< _doors.size(); i++){
                ROS_INFO("Door %d ",_doors[i]);
        }
    }

    void CreateNewTasks(int num){
        for(size_t i = 0; i< _doors.size(); i++){
            ROS_INFO("Door %d ",_doors[i]);
        }
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

    // vector<vector<TaskInTable>> MakeTaskSerie(vector<TaskInTable> &tasks){
    //     vector<vector<TaskInTable>> series;
    //     for(vector<TaskInTable>::iterator it = tasks.begin(); it != tasks.end(); ){
    //         if(it->dependency==0){ // Find task with no dependency
    //             vector<TaskInTable> s;
    //             s.push_back(*it);
    //             series.push_back(s);
    //             it = tasks.erase(it); // Erase it from task vector when finish calculating
    //         }else{
    //             it++;
    //         }
    //     }
    //     ROS_INFO("Create %ld series finished. Distribute %ld tasks",series.size(),tasks.size());

    //     while(!tasks.empty()){
    //         for(vector<TaskInTable>::iterator it = tasks.begin(); it != tasks.end();){
    //             const int d = it->dependency; // Find serie
    //             vector<vector<TaskInTable>>::iterator serieIt = find_if(series.begin(),series.end(),
    //                 [d](const vector<TaskInTable>& s) ->bool {return s.back().taskId == d;}
    //             );
    //             if(serieIt == series.end()){
    //                 ROS_INFO("Task %d dependence on unknown task",it->taskId);
    //                 _sc.UpdateTaskStatus(it->taskId,"Error"); // If a task is depend on unknown task, set it to error 
    //             }else{ 
    //                 serieIt->push_back(*it);
    //                 ROS_INFO("Put task %d in serie %ld",it->taskId,serieIt - series.begin());
    //             }
    //             it = tasks.erase(it); // Erase it from task vector when finish calculating
    //         }
    //     }
    //     return series;
    // }

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

    void CalculateLargeTasksCost(geometry_msgs::Pose robotPose,vector<LargeTask>& lts){
        ROS_INFO_STREAM("Large_task_id Battery WaitTime Open_possibility Priority   Cost");
        ROS_INFO("-----------------------------------------------------------------------------");
        ros::Time now = ros::Time::now();
        
        for( auto lit = lts.begin(); lit != lts.end(); lit++){
            lit->battery = CalculateLargeTaskBatteryConsumption(robotPose,lit->tasks);
            lit->waitingTime = lit->tasks.begin()->second.header.stamp - now;
            lit->cost = CF.A + CF.B *lit->battery + CF.C * lit->waitingTime.toSec() + CF.D * lit->openPossibility + CF.E * lit->priority;
            ROS_INFO("%d        %.3f   %.3f   %.3f  %d  %3f",lit->largeTaskId,lit->battery,lit->waitingTime.toSec(), lit->openPossibility,lit->priority,lit->cost);
        }
    }

    void SortLargeTasksWithCost(vector<LargeTask>& lts){
        sort(lts.begin(),lts.end(),
        [](const LargeTask&lt1,const LargeTask& lt2)->bool{
            return lt1.cost > lt2.cost;
        });
    }

    // void CalculateCostForSerie(vector<vector<TaskInTable>> &series,geometry_msgs::Pose robotPose){
    //     double batteryConsumption = 0;
    //     int waitTime = 0;
    //     ros::Time now = ros::Time::now();
    //     geometry_msgs::Pose start;

    //     ROS_INFO_STREAM("serie id type     Target_id Priority Open_pos Battery WaitTime Cost");
    //     ROS_INFO("-----------------------------------------------------------------------------");

    //     for(vector<vector<TaskInTable>>::iterator it1 = series.begin(); it1 != series.end();it1++){
    //         vector<TaskInTable>::iterator it2 = it1->begin();
    //         batteryConsumption = CalculatSmallTaskBatteryConsumption(robotPose,it2->goal.pose);
    //         waitTime = it2->goal.header.stamp.sec - now.sec;
    //         it2->cost =   10 + 10 * batteryConsumption + 0.1 * waitTime + (-10) * it2->openPossibility + (-1) * it2->priority; 
    //         start = it2->goal.pose;
    //         ROS_INFO("[serie %ld]  %d  %s %d   %d  %.3f  %.3f  %.3f  %.3f",it1-series.begin(),it2->taskId,it2->taskType.c_str(),it2->targetId,it2->priority,it2->openPossibility,batteryConsumption,waitTime,it2->cost);
    //         for(it2++; it2!=it1->end(); it2++){
    //             batteryConsumption = CalculatSmallTaskBatteryConsumption(start,it2->goal.pose);
    //             waitTime = it2->goal.header.stamp.sec - now.sec;
    //             it2->cost =   10 + 10 * batteryConsumption + 0.1 * waitTime + (-10) * it2->openPossibility + (-1) * it2->priority; 
    //             start = it2->goal.pose;
    //             ROS_INFO("         %d  %s %d   %d  %.3f  %.3f  %d  %.3f",it2->taskId,it2->taskType.c_str(),it2->targetId,it2->priority,it2->openPossibility,batteryConsumption,waitTime,it2->cost);
    //         }
    //     }
    // }

    
    // void FilterSerie(vector<vector<TaskInTable>>& series){
    //     for(vector<vector<TaskInTable>>::iterator it1 = series.begin(); it1 != series.end();it1++){
    //         for(vector<TaskInTable>::iterator it2 = it1->begin(); it2!=it1->end(); it2++){
    //             if(it2->cost > COST_LIMIT){
    //                 series.erase(it1); // TODO
    //             }
    //         }
    //     }
    // }

    // void SortSerieWithCost(vector<vector<TaskInTable>>& series){
    //     std::sort(series.begin(),series.end(),
    //     [](const vector<TaskInTable>& s1, const vector<TaskInTable>& s2)->bool
    //     {
    //             return s1.back().cost > s2.back().cost; // compare last task in serie
    //     });
    // }

    // vector<TaskInTable> CalculateCostofTasks(vector<TaskInTable> &tasks, geometry_msgs::Pose robotPose){
    //     ros::Time now = ros::Time::now();
    //     vector<TaskInTable> tasksWithCost;
    //     double batteryConsumption = 0,openPossibility = 0;
    //     int priority = 0,waitTime = 0;
       
    //     ROS_INFO_STREAM("id type     Target_id Priority Open_pos Battery WaitTime Cost");
    //     ROS_INFO("-----------------------------------------------------------------------------");
    //     for(vector<TaskInTable>::iterator it = tasks.begin(); it != tasks.end(); ){
    //         if(it->dependency==0){ // Find task with no dependency
    //             batteryConsumption = CalculatSmallTaskBatteryConsumption(robotPose,it->goal.pose);
    //             waitTime = it->goal.header.stamp.sec - now.sec;
    //             openPossibility = it->openPossibility;
    //             priority = it->priority;
    //             it->cost =   10 + 10 * batteryConsumption + 0.1 * waitTime + (-10) * openPossibility + (-1) * priority; 
    //             tasksWithCost.push_back(*it);   
    //             ROS_INFO("%d  %s %d   %d  %.3f  %.3f  %d  %.3f",it->taskId,it->taskType.c_str(),it->targetId,priority,openPossibility,batteryConsumption,waitTime,it->cost);

    //             it = tasks.erase(it); // Erase it from task vector when finish calculating            
    //         }else{
    //             it++;
    //         }
    //     }

    //     ROS_INFO_STREAM("Basic task done");
    //    while(!tasks.empty()){
    //         for(vector<TaskInTable>::iterator it = tasks.begin(); it != tasks.end();){
    //             const int d = it->dependency;
    //             vector<TaskInTable>::iterator dependencyTaskIt = find_if(tasksWithCost.begin(),tasksWithCost.end(),
    //                 [d](const TaskInTable& t) ->bool {return t.taskId == d;}
    //             );
    //             if(dependencyTaskIt == tasksWithCost.end()){
    //                 ROS_INFO("Task %d dependence on unknown task",it->taskId);
    //                 _sc.UpdateTaskStatus(it->taskId,"Error"); // If a task is depend on unknown task, set it to error 
    //             }else{ 
    //                 // If task is depend on another task
    //                 batteryConsumption = CalculatSmallTaskBatteryConsumption(dependencyTaskIt->goal.pose,it->goal.pose);
    //                 waitTime = it->goal.header.stamp.sec - now.sec;
    //                 openPossibility = it->openPossibility * dependencyTaskIt->openPossibility;
    //                 priority = it->priority;
    //                 it->cost =  dependencyTaskIt->cost +  1.0 * batteryConsumption + 0.2 * waitTime + (-100) * openPossibility + (-10) * priority; 
    //                 tasksWithCost.push_back(*it);

    //                  ROS_INFO("%d  %s %d   %d  %.3f  %.3f  %d  %.3f",it->taskId,it->taskType.c_str(),it->targetId,priority,openPossibility,batteryConsumption,waitTime,it->cost);

    //             }
    //                 it = tasks.erase(it); // Erase it from task vector when finish calculating
    //         }
    //    }

    //    return tasksWithCost;
    // }


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

    // vector<TaskInTable> SelectBestTaskSiere(geometry_msgs::Pose robotPose){
    //         vector<TaskInTable> v;
    //         vector<TaskInTable> siere;
    //         vector<vector<TaskInTable> > sieres;
    //         // TaskInTable bt;
    //         v = _sc.QueryRunableExecuteTasks();  // find if there are execute task    
    //         ROS_INFO_STREAM("found "<<v.size()<<" execute tasks");

    //         if(v.size() != 0 ){
    //             sieres = MakeTaskSerie(v);
    //             CalculateCostForSerie(sieres,robotPose);
    //             // FilterSerie(series);
    //             ROS_INFO_STREAM("Calculate cost finish");
    //             SortSerieWithCost(sieres);
    //             ROS_INFO_STREAM("Sort cost finish");
    //             // FilterTask(v);
    //         }

    //         if(sieres.size() == 0){ // after filter, if there is no execute task, gather inviroment
    //             while((v = _sc.QueryRunableGatherEnviromentInfoTasks()).size() == 0){  // if no execute task, create some gather enviroment info task
    //                 CreateNewTasks(10);    
    //                 ros::Duration(2).sleep();            
    //             }
    //             ROS_INFO_STREAM("found "<<v.size()<<"gather enviroment info tasks");
    //             sieres = MakeTaskSerie(v);
    //             CalculateCostForSerie(sieres,robotPose);
    //             // FilterSerie(series);
    //             ROS_INFO("Calculate cost finish");
    //             SortSerieWithCost(sieres);
    //             ROS_INFO("Sort cost finish. series size %ld",sieres.size());
               
    //             // v = CalculateCostofTasks(v,robotPose); // calculate cost
                
    //             // SortTaskWithCost(v);
    //             // bt = v.back();
    //         }
            
    //         // bt =  series.back().back();
    //         // TaskInTable bt = v.back();
    //         // ROS_INFO("series size %ld",sieres.size());
    //         siere = sieres.back();
        
    //         return siere;
        
    // }

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