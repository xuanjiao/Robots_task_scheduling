#include "ros/ros.h"
// #include "robot_navigation/make_task.h"
#include "robot_navigation/GetATask.h"
#include <boost/thread/mutex.hpp>
#include "robot_navigation/GoToTargetAction.h"
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>
#include <vector>
#include "util.h"
#include "sql_client.h"
#include <tuple>
#include <string>
#include <queue>

#define CHECK_DB_PERIOD 60
#define COST_LIMIT 1000

typedef actionlib::SimpleActionClient<robot_navigation::GoToTargetAction> GoToTargetActionClient;                                                                                                                                                                                  ;
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

class CentralizedPool{

public:
 CentralizedPool():
        _sc("centralized_pool","pass"),
        _gac("GoToTargetAction",true) //  spins up a thread to service this action's subscriptions. 
    {
        init();
        // create_gather_info_tasks(10);
        CreateASampleExecuteTask();
    }

    void init(){
	    ros::Duration(1).sleep();
        ROS_INFO_STREAM("Current office time: "<<Util::time_str(ros::Time::now()));
        // _ts = _nh.advertiseService("make_task",&CentralizedPool::process_robot_request,this);
        _ts = _nh.advertiseService("GetATask",&CentralizedPool::WhenRobotRequestTask,this);
        _pc = _nh.serviceClient<nav_msgs::GetPlan>("/tb3_0/move_base/NavfnROS/make_plan"); 
        _sqlMtx.lock();
        _sc.TruncateTable("tasks");     
        _sqlMtx.unlock();
    }

    void CreateASampleExecuteTask(){
        Task t;
        t.task_type = "ExecuteTask";
        t.priority = 4;
        t.goal.pose.position.x = -1.43263357857;
        t.goal.pose.position.y = -3.65467268359;
        t.goal.pose.orientation.z = 0.42598373778;
        t.goal.pose.orientation.w = 0.904730819165;
        t.goal.header.stamp = ros::Time::now()+ros::Duration(20);
        t.goal.header.frame_id = "map";
        _sqlMtx.lock();
        t.target_id = _sc.InsertATargetAssignId(t.goal);
        _sc.InsertATaskAssignId(t);       
        _sqlMtx.unlock();
    }


    // call back when receive robot request for task //
    bool WhenRobotRequestTask(robot_navigation::GetATask::Request &req, 
        robot_navigation::GetATask::Response &res){  
        ROS_INFO_STREAM("Receive request from robot\n"<<req);
        
        if(req.batteryLevel < 20){ // charging
            ResponceChargingTask(req,res);
        }else{
            ResponceTaskWithLowestCost(req,res);
        }
        return true;
    }

     // call back when receive a door status from robot 
    void WhenReceiveInfoFromRobot(const robot_navigation::GoToTargetFeedbackConstPtr &feedback){
        ROS_INFO_STREAM("Update door list and possibility table. [Time]:"<<
            Util::time_str(feedback->m_time)<<" [Door] " << to_string(feedback->door_id) << 
            " [status] "<<to_string(feedback->door_status));
        _sqlMtx.lock();
        _sc.InsertDoorStatusRecord(feedback->door_id,feedback->m_time,feedback->door_status); 
        _sc.UpdateOpenPossibilities(feedback->door_id,feedback->m_time);
        _sqlMtx.unlock();
    }

    // Call back when robot receive a goal                                                                                                       all when robot receive a goal
    void WhenActionActive(){
        ROS_INFO_STREAM("Goal arrived to robot");
    }   

    // Call when receive a complet event from robot
    void WhenRobotFinishGoal(const actionlib::SimpleClientGoalState& state,
           const robot_navigation::GoToTargetResult::ConstPtr &result){
        ROS_INFO_STREAM("State "<<state.toString()<<" result = "<<*result);
         _sqlMtx.lock();
         if(state == actionlib::SimpleClientGoalState::SUCCEEDED){  
             _sc.UpdateTaskStatus(result->task_id,"RanToCompletion");// Mark task as RanToCompletion
             ROS_INFO("Mark task %d as completed",result->task_id);
        }else{
            // change task status from Running to ToReRun, increase priority 3 and increase 200s start time 
            _sc.UpdateReturnedTask(result->task_id,3,ros::Duration(200));
        }
        _sqlMtx.unlock();
    }

    // Create Respon to robot
    void ResponceChargingTask(robot_navigation::GetATask::Request &req,robot_navigation::GetATask::Response &res ){
        _sqlMtx.lock();
        std::map<int,geometry_msgs::Pose> map = _sc.QueryAvailableChargingStations();
        geometry_msgs::Pose rp = req.pose;
        ros::Time now = ros::Time::now();
        if(map.size()==0){
            ROS_INFO_STREAM("All charging station are busy");
            res.hasTask = false;
        }
        res.hasTask = true;
        std::pair<int,double> best; // best charging station (id,distance) 
        best.second = 1000;  
            for(auto i : map){
            double dist = calculate_distance(i.second,rp);
            if(dist<best.second){
                best.first = i.first;
                best.second = dist;
            }
        }
        
        Task bt;
        bt.task_type = "Charging";
        bt.priority = 5;
        bt.goal.header.stamp = now + ros::Duration(10); // create a charging task that start after 10s
        bt.goal.header.frame_id = "map";
        bt.goal.pose = map[best.first];

        
        _sc.InsertATaskAssignId(bt); // insert task into database

        SendRobotActionGoal(bt); // send goal to robot
        _sqlMtx.unlock();
        ROS_INFO_STREAM("Send robot a charging task");
    }

    void ResponceTaskWithLowestCost(robot_navigation::GetATask::Request &req,robot_navigation::GetATask::Response &res){
        ros::Time cur_time = ros::Time::now();
        ROS_INFO_STREAM("current time =  "<<cur_time);
        _sqlMtx.lock();
         _sc.PrintTable("tasks");
        ROS_INFO("Handle expired tasks...");
        _sc.UpdateExpiredTask(cur_time+ ros::Duration(20));
        ROS_INFO("Handle expired tasks finished");
        _sc.PrintTable("tasks"); 
        std::vector<Task> v;

    
        v = _sc.QueryRunableExecuteTasksBeforeDateTime(cur_time + ros::Duration(CHECK_DB_PERIOD));  // find if there are execute task    
        ROS_INFO_STREAM("found "<<v.size()<<" execute tasks");
        if(v.size() == 0){
            while((v = _sc.QueryRunableGatherEnviromentInfoTasks()).size() == 0){  // if no execute task, create some gather enviroment info task
                _sc.InsertMultipleGatherInfoTasks(5,cur_time + ros::Duration(20),ros::Duration(50));
                ROS_INFO_STREAM("Create 5 tasks");
            }
            ROS_INFO_STREAM("found "<<v.size()<<"gather enviroment info tasks");
        }
        Task bt = GetBestTask(v,req.pose,cur_time,req.batteryLevel);
        ROS_INFO_STREAM("Best task id = "<<bt.task_id<<" ,cost = "<< fixed << setprecision(3) << setw(6)<< bt.cost);
        res.hasTask = true;
        _sc.UpdateTaskStatus(bt.task_id,"WaitingToRun");
        SendRobotActionGoal(bt); 
        _sqlMtx.unlock();
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
    
    Task GetBestTask(vector<Task> & v,geometry_msgs::Pose robot_pose,ros::Time t,double battery){
        ROS_INFO_STREAM("Task_id  Task_type Target_id Priority Open_pos Distance Sec_diff Cost");
        ROS_INFO("-----------------------------------------------------------------------------");
        for(auto &i :v){
               CostFunction cf(
                   calculate_distance(i.goal.pose,robot_pose),
                   i.goal.header.stamp.sec - t.sec,
                   i.priority,
                   i.open_pos,
                   battery
                );
                i.cost = cf._cost;

                ROS_INFO_STREAM(
                    setw(3) <<i.task_id <<" "<<setw(5)<<i.task_type <<setw(4) << i.target_id << setw(2) << i.priority << setprecision(3)<< setw(4) << 
                    i.open_pos << " " << setprecision(3)<< setw(5) << cf._distance << " "<<fixed<<setprecision(3) << setw(10) <<cf._sec_diff<< " " <<fixed << setprecision(3) << setw(6) <<i.cost 
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
               ROS_INFO_STREAM(it->task_id<<" "<<it->cost<<" (cost > " << to_string(COST_LIMIT) << " delete)");
               v.erase(it);
            }else{
                ROS_INFO_STREAM(it->task_id<<" "<<it->cost);
            }
        }
        return v.back();
    }

    // Send robot new task 
    void SendRobotActionGoal(Task &bt){
        robot_navigation::GoToTargetGoal g;
        g.goal = bt.goal;
        g.target_id = bt.target_id;
        g.task_type = bt.task_type;
        g.task_id = bt.task_id;
        
        _gac.sendGoal(g,
                boost::bind(&CentralizedPool::WhenRobotFinishGoal,this,_1,_2),
                boost::bind(&CentralizedPool::WhenActionActive,this),
                boost::bind(&CentralizedPool::WhenReceiveInfoFromRobot,this,_1)
        );
        ROS_INFO_STREAM("Send a goal\n"<<g);
    }

private:
    ros::ServiceServer _ts;
    ros::ServiceClient _pc;
    GoToTargetActionClient _gac;
    SQLClient _sc;
    boost::mutex _sqlMtx;
    ros::NodeHandle _nh;
};

int main(int argc, char** argv){
    ros::init(argc,argv,"centralized_poor");
    CentralizedPool pool;
    ros::spin(); // block program
}