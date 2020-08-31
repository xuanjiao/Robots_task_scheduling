#include "ros/ros.h"
#include "robot_navigation/GetATask.h"
#include "robot_navigation/RunTaskAction.h"
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>
#include <vector>
#include "util.h"
#include "sql_client.h"
#include "task_manager.h"
#include <string>

#define CHECK_DB_PERIOD 10
#define CHARGING_THRESHOLD 97

typedef actionlib::SimpleActionClient<robot_navigation::RunTaskAction> RunTaskActionClient;                                                                                                                                                                                  ;

class CentralizedPool{

public:
 CentralizedPool(SQLClient& sc,ros::NodeHandle &nh,TaskManager &tm):_nh(nh),_tm(tm)
        // _gac("/tb3_0/RunTaskAction",true) //  spins up a thread to service this action's subscriptions. 
    {
        _cv.push_back(new RunTaskActionClient("/tb3_0/run_task_action",true));
        _cv.push_back(new RunTaskActionClient("/tb3_1/run_task_action",true));
        _cv.push_back(new RunTaskActionClient("/tb3_2/run_task_action",true));
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
    }

    // call back when receive robot request for task //
    bool WhenRobotRequestTask(robot_navigation::GetATask::Request &req, 
        robot_navigation::GetATask::Response &res){  
        ROS_INFO("REQUEST from Robot %d (%f,%f) battery %f",req.robotId,req.pose.position.x,req.pose.position.y,req.batteryLevel);
        
        ROS_INFO_STREAM("Current time: "<<Util::time_str(ros::Time::now()));
        if(req.batteryLevel <= CHARGING_THRESHOLD){ // need charging
            SmallTask bt = _tm.CreateBestChargingTask(req.pose);
            SendRobotSmallTask(bt,req.robotId); // send goal to robot
        }else{
            LargeExecuteTask lt = _tm.SelectExecutetask(req.pose);
            if(lt.smallTasks.size()>0){
                SendRobotLargeTask(lt,req.robotId); 
            }else{
                SmallTask bt = _tm.CreateBestEnviromentTask(req.pose);
                SendRobotSmallTask(bt,req.robotId);
            }
        }
        res.hasTask = true;
        return true;
    }

     // call back when receive a door status from robot 
    void WhenReceiveInfoFromRobot(const robot_navigation::RunTaskFeedbackConstPtr &feedback){
        ROS_INFO("FEEDBACK from Robot %d : Time %s isOpen %d",feedback->robotId,Util::time_str(feedback->measureTime).c_str(),feedback->doorStatus);
        TaskFeedback fb;
        fb.doorId = feedback->doorId;
        fb.doorStatus = feedback->doorStatus;
        fb.measureTime = feedback->measureTime;
        _tm.HandleTaskFeedback(fb);
    }

    // Call when receive a complet event from robot
    void WhenRobotFinishGoal(const actionlib::SimpleClientGoalState& state,
           const robot_navigation::RunTaskResult::ConstPtr &result){
        ROS_INFO("RESULT from robot %d: %s %s ",result->robotId,result->taskType.c_str() ,state.toString().c_str());
        TaskResult rs;
        rs.isCompleted = (state == actionlib::SimpleClientGoalState::SUCCEEDED)?true:false;
        rs.description = result->description;
        rs.taskIds = result->taskIds;
        rs.taskType = result->taskType;
        // ROS_INFO_STREAM("task result "<<rs.isCompleted<<rs.taskType<<rs.description);
        _tm.HandleTaskResult(rs);
    }

    // Send robot new task 
    void SendRobotSmallTask(SmallTask &bt,int robotId){
        ROS_INFO_STREAM("Send task to robot"<<robotId<<" "<<bt.getTaskInfo());
        robot_navigation::RunTaskGoal g;
        g.goals.push_back(bt.goal);
        g.taskIds.push_back(bt.taskId);
        g.targetIds.push_back(bt.targetId);
        g.taskType = bt.taskType;
        _acMtx.lock();
        _cv[robotId]->sendGoal(g,
                boost::bind(&CentralizedPool::WhenRobotFinishGoal,this,_1,_2),
                actionlib::SimpleActionClient<robot_navigation::RunTaskAction>::SimpleActiveCallback(),
                // boost::bind(&CentralizedPool::WhenActionActive,this),
                boost::bind(&CentralizedPool::WhenReceiveInfoFromRobot,this,_1)
        );
      //  ROS_INFO_STREAM(g);
        _acMtx.unlock();
        _tm.AfterSendingTask(bt.taskId,robotId); // Update task status and robot id column in task table
    }

    void SendRobotLargeTask(LargeExecuteTask& lt,int robotId){
        robot_navigation::RunTaskGoal g;
        int taskId;
        ROS_INFO_STREAM("Send to robot "<<robotId<<" "<<lt.getTaskInfo());
        g.taskType = lt.taskType;

        for(auto it = lt.smallTasks.begin();it !=  lt.smallTasks.end(); it++ ){
            taskId = it->first;
            g.goals.push_back(it->second.goal);
            g.taskIds.push_back(taskId);
            g.targetIds.push_back(it->second.targetId);
           
            _tm.AfterSendingTask(taskId,robotId); 
        }

        _acMtx.lock();
        _cv[robotId]->sendGoal(g,
                boost::bind(&CentralizedPool::WhenRobotFinishGoal,this,_1,_2),
                actionlib::SimpleActionClient<robot_navigation::RunTaskAction>::SimpleActiveCallback(),
                // boost::bind(&CentralizedPool::WhenActionActive,this),
                boost::bind(&CentralizedPool::WhenReceiveInfoFromRobot,this,_1)
        );
        // ROS_INFO_STREAM(g);
        _acMtx.unlock();
    }

private:
    ros::ServiceServer _ts;
    boost::mutex _acMtx;
    std::vector<RunTaskActionClient*> _cv;
    ros::NodeHandle &_nh;
    TaskManager &_tm;
};

int main(int argc, char** argv){
    ros::init(argc,argv,"centralized_poor");
    ros::NodeHandle nodeHandle;
    SQLClient sqlClient("centralized_pool","pass");
    CostCalculator cc(nodeHandle);
    TaskManager taskManager(sqlClient,cc);
    CentralizedPool pool(sqlClient,nodeHandle,taskManager);

    ros::spin(); // block program
}