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
#include "TaskManager.h"
#include <tuple>
#include <string>
#include <queue>

#define CHECK_DB_PERIOD 60


typedef actionlib::SimpleActionClient<robot_navigation::GoToTargetAction> GoToTargetActionClient;                                                                                                                                                                                  ;

class CentralizedPool{

public:
 CentralizedPool(SQLClient& sc,ros::NodeHandle &nh,TaskManager tm):_sc(sc),_nh(nh),_tm(tm)
        // _gac("/tb3_0/GoToTargetAction",true) //  spins up a thread to service this action's subscriptions. 
    {
        _cv.push_back(new GoToTargetActionClient("/tb3_0/GoToTargetAction",true));
        _cv.push_back(new GoToTargetActionClient("/tb3_1/GoToTargetAction",true));
        _cv.push_back(new GoToTargetActionClient("/tb3_2/GoToTargetAction",true));
        init();
    }
    ~CentralizedPool(){
        for(size_t i = 0 ; i< _cv.size(); i++){
            delete _cv[i];
        }
    }

    void init(){
	    ros::Duration(1).sleep();
        ROS_INFO_STREAM("Current office time: "<<Util::time_str(ros::Time::now()));
        // _ts = _nh.advertiseService("make_task",&CentralizedPool::process_robot_request,this);
        _ts = _nh.advertiseService("/GetATask",&CentralizedPool::WhenRobotRequestTask,this);
       
        _sqlMtx.lock();
        _sc.TruncateTable("tasks");     
        _sqlMtx.unlock();
    }

    // call back when receive robot request for task //
    bool WhenRobotRequestTask(robot_navigation::GetATask::Request &req, 
        robot_navigation::GetATask::Response &res){  
        ROS_INFO("Robot %d request a task",req.robotId);
        ROS_INFO_STREAM(req);
        if(req.batteryLevel < 20){ // charging
            ResponceChargingTask(req,res);
        }else{
            ResponceTaskWithLowestCost(req,res);
        }
        return true;
    }

     // call back when receive a door status from robot 
    void WhenReceiveInfoFromRobot(const robot_navigation::GoToTargetFeedbackConstPtr &feedback){
        // ROS_INFO_STREAM("Robot "<<feedback->robotId <<" Feedback: [Time]:"<<
        //     Util::time_str(feedback->measureTime)<<" [Door] " << to_string(feedback->doorId) << 
        //     " [status] "<<to_string(feedback->doorStatus));

        ROS_INFO("Robot %d feedback: ",feedback->robotId);
        ROS_INFO_STREAM(*feedback);
        _sqlMtx.lock();
        int r = _sc.InsertDoorStatusRecord(feedback->doorId,feedback->measureTime,feedback->doorStatus); 
        int u = _sc.UpdateOpenPossibilities(feedback->doorId,feedback->measureTime);
        ROS_INFO("Insert %d record, update %d rows in possibility table",r,u);
        _sqlMtx.unlock();
    }

    // // Call back when robot receive a goal                                                                                                       all when robot receive a goal
    // void WhenActionActive(){
    //     ROS_INFO_STREAM("Goal arrived to robot");
    // }   

    // Call when receive a complet event from robot
    void WhenRobotFinishGoal(const actionlib::SimpleClientGoalState& state,
           const robot_navigation::GoToTargetResult::ConstPtr &result){
        ROS_INFO("Robot %d result:",result->robotId);
        ROS_INFO_STREAM(*result); 
         if(state == actionlib::SimpleClientGoalState::SUCCEEDED){  
             ROS_INFO("Task Succedd. Update %d task status",_sc.UpdateTaskStatus(result->taskId,"RanToCompletion"));
        }else{
            // change task status from Running to ToReRun, increase priority 3 and increase 200s start time 
             ROS_INFO("Task failed. Update %d returned task ",_sc.UpdateReturnedTask(result->taskId,3,ros::Duration(200)));
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
            double dist = _tm.calculate_distance(i.second,rp);
            if(dist<best.second){
                best.first = i.first;
                best.second = dist;
            }
        }
        
        Task bt;
        bt.taskType = "Charging";
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
        ROS_INFO("Handled %d expired tasks",_sc.UpdateExpiredTask(cur_time+ ros::Duration(20)));
        _sc.PrintTable("tasks"); 
        std::vector<Task> v;

        v = _sc.QueryRunableExecuteTasks();  // find if there are execute task    
        ROS_INFO_STREAM("found "<<v.size()<<" execute tasks");
        if(v.size() == 0){
            while((v = _sc.QueryRunableGatherEnviromentInfoTasks()).size() == 0){  // if no execute task, create some gather enviroment info task
                _tm.CreateNewTasks(10);                
            }
            ROS_INFO_STREAM("found "<<v.size()<<"gather enviroment info tasks");
        }
        Task bt = _tm.GetBestTask(v,req.pose,cur_time,req.batteryLevel);
        ROS_INFO_STREAM("Best task id = "<<bt.taskId<<" ,cost = "<< fixed << setprecision(3) << setw(6)<< bt.cost);
        res.hasTask = true;
        _sc.UpdateTaskStatus(bt.taskId,"WaitingToRun");
        bt.robotId = req.robotId;
        SendRobotActionGoal(bt); 
        _sqlMtx.unlock();
    }


    // Send robot new task 
    void SendRobotActionGoal(Task &bt){
        robot_navigation::GoToTargetGoal g;
        g.goals.push_back(bt.goal);
        g.targetId= bt.targetId;
        g.taskType = bt.taskType;
        g.taskId = bt.taskId;
        g.robotId = bt.robotId;
        _cv[bt.robotId]->sendGoal(g,
                boost::bind(&CentralizedPool::WhenRobotFinishGoal,this,_1,_2),
                actionlib::SimpleActionClient<robot_navigation::GoToTargetAction>::SimpleActiveCallback(),
                // boost::bind(&CentralizedPool::WhenActionActive,this),
                boost::bind(&CentralizedPool::WhenReceiveInfoFromRobot,this,_1)
        );
        ROS_INFO_STREAM("Send a goal\n"<<g);
    }

private:
    ros::ServiceServer _ts;
    
    // GoToTargetActionClient _gac;
    std::vector<GoToTargetActionClient*> _cv;
    SQLClient &_sc;
    boost::mutex _sqlMtx;
    ros::NodeHandle &_nh;
    TaskManager &_tm;
};

int main(int argc, char** argv){
    ros::init(argc,argv,"centralized_poor");
    ros::NodeHandle nodeHandle;
    SQLClient sqlClient("centralized_pool","pass");
    TaskManager taskManager(sqlClient,nodeHandle);
    CentralizedPool pool(sqlClient,nodeHandle,taskManager);

    ros::spin(); // block program
}