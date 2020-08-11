#include "ros/ros.h"
// #include "robot_navigation/make_task.h"
#include "robot_navigation/GetATask.h"
#include "robot_navigation/GoToTargetAction.h"
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>
#include <vector>
#include "util.h"
#include "sql_client.h"
#include "task_manager.h"
#include <tuple>
#include <string>
#include <queue>

#define CHECK_DB_PERIOD 10


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
        _ts = _nh.advertiseService("/GetATask",&CentralizedPool::WhenRobotRequestTask,this);
        // _sc.TruncateTable("tasks");     
    }

    // call back when receive robot request for task //
    bool WhenRobotRequestTask(robot_navigation::GetATask::Request &req, 
        robot_navigation::GetATask::Response &res){  
        ROS_INFO("Robot %d request a task",req.robotId);
        
        ROS_INFO_STREAM(req);
        ros::Time cur_time = ros::Time::now();
        ROS_INFO_STREAM("current time =  "<<cur_time);   
        if(req.batteryLevel < 20){ // charging
            // ResponceChargingTask(req,res);
        }else{
            vector<TaskInTable> siere = _tm.SelectBestTaskSiere(req.pose);
            res.hasTask = true;
            SendRobotMultipleTargetActionGoal(siere,req.robotId); 
            // ResponceTaskWithLowestCost(req,res);
        }
        return true;
    }

     // call back when receive a door status from robot 
    void WhenReceiveInfoFromRobot(const robot_navigation::GoToTargetFeedbackConstPtr &feedback){
        ROS_INFO("Robot %d feedback: ",feedback->robotId);
        ROS_INFO_STREAM(*feedback);
        
        int r = _sc.InsertDoorStatusRecord(feedback->doorId,feedback->measureTime,feedback->doorStatus); 
        int u = _sc.UpdateOpenPossibilities(feedback->doorId,feedback->measureTime);
        ROS_INFO("Insert %d record, update %d rows in possibility table",r,u);
        
    }

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
        
    }

    // Create Respon to robot
    void ResponceChargingTask(robot_navigation::GetATask::Request &req,robot_navigation::GetATask::Response &res ){
        
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
            double dist = _tm.CalculatSmallTaskBatteryConsumption(rp,i.second);
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
        bt.goal.pose = map[best.first];   
        _sc.InsertATaskAssignId(bt); // insert task into database
        SendRobotActionGoal(bt); // send goal to robot
        
        ROS_INFO_STREAM("Send robot a charging task");
    }

    void ResponceTaskWithLowestCost(robot_navigation::GetATask::Request &req,robot_navigation::GetATask::Response &res){

    }


    // Send robot new task 
    void SendRobotActionGoal(TaskInTable &bt){
        robot_navigation::GoToTargetGoal g;
        g.goals.push_back(bt.goal);
        g.targetId= bt.targetId;
        g.taskType = bt.taskType;
        g.taskId = bt.taskId;
        g.robotId = bt.robotId;

        _acMtx.lock();
        _cv[bt.robotId]->sendGoal(g,
                boost::bind(&CentralizedPool::WhenRobotFinishGoal,this,_1,_2),
                actionlib::SimpleActionClient<robot_navigation::GoToTargetAction>::SimpleActiveCallback(),
                // boost::bind(&CentralizedPool::WhenActionActive,this),
                boost::bind(&CentralizedPool::WhenReceiveInfoFromRobot,this,_1)
        );
        ROS_INFO_STREAM("Send a goal\n"<<g);
        _acMtx.unlock();
    }

    void SendRobotMultipleTargetActionGoal(vector<TaskInTable> &siere,int robotId){
        robot_navigation::GoToTargetGoal g;
        ROS_INFO("Send best siere to robot: ");
        for(std::vector<TaskInTable>::iterator it = siere.begin();it != siere.end(); it++ ){
            ROS_INFO("task %d ",it->taskId);
            g.goals.push_back(it->goal);
            _sc.UpdateTaskStatus(it->taskId,"Running");
            _sc.UpdateTaskRobotId(it->taskId,robotId);
        }
        
        g.targetId= siere.back().targetId;
        g.taskType = siere.back().taskType;
        g.taskId = siere.back().taskId;
        g.robotId = robotId;

        _acMtx.lock();
        _cv[robotId]->sendGoal(g,
                boost::bind(&CentralizedPool::WhenRobotFinishGoal,this,_1,_2),
                actionlib::SimpleActionClient<robot_navigation::GoToTargetAction>::SimpleActiveCallback(),
                // boost::bind(&CentralizedPool::WhenActionActive,this),
                boost::bind(&CentralizedPool::WhenReceiveInfoFromRobot,this,_1)
        );
        ROS_INFO_STREAM("Send a goal\n"<<g);
        _acMtx.unlock();
    }

private:
    ros::ServiceServer _ts;
    
    // GoToTargetActionClient _gac;
    boost::mutex _acMtx;
    std::vector<GoToTargetActionClient*> _cv;
    SQLClient &_sc;
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