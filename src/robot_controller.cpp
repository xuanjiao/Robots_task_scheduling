#include "ros/ros.h"
#include "robot_navigation/sensor_data.h"
// #include "robot_navigation/make_task.h"
#include "robot_navigation/GetATask.h"
#include "robot_navigation/GoToTargetAction.h"
#include <cmath>
#include <sstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <string> 
#include "util.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#define SENSOR_RANGE 1
using namespace std;

class RobotController{

public:
    RobotController(int robotId):
    _robotId(robotId),
    _mbc("/tb3_" + std::to_string(robotId) + "/move_base", true),
    _gas(_nh,"GoToTargetAction",boost::bind(&RobotController::ExecuteCallback,this,_1),false),
    _movLock(_mtx),
    _battery(100)
    {
        Init();
        RequestTask();
    }

    void Init(){

        ROS_INFO_STREAM("Robot "<<_robotId << " start initializing..");
        _ss = _nh.subscribe<robot_navigation::sensor_data>("sensor_data",100,&RobotController::ListenSensorCallback,this);      
        _tc = _nh.serviceClient<robot_navigation::GetATask>("GetATask");
       
        while(!_mbc.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        _gas.start();  // start receive goal from server
        ROS_INFO_STREAM("Robot "<<_robotId << " finish initializing");
    }

    void RequestTask(){
  
        boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> sharedPtr = NULL;
        
        ROS_INFO_STREAM("Get current position...");
        while(ros::ok()){
            // try to get its current location
            sharedPtr =  ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/tb3_"+ to_string(_robotId) +"/amcl_pose",_nh);
            if(sharedPtr == NULL){
                ROS_INFO_STREAM("Failed to get current position");
            }else{
                _cp.pose = sharedPtr->pose.pose;
                ROS_INFO_STREAM("Robot "<< _robotId <<" "<<Util::pose_str(_cp.pose)); 
                break;
            }
            ros::Duration(5).sleep();
        }
  
        // Use location to request a task
        robot_navigation::GetATask srv;
        srv.request.batteryLevel=  _battery;
        srv.request.pose = sharedPtr->pose.pose;
        srv.request.robotId = _robotId;
        
        while(ros::ok()){
            if(!_tc.call(srv)){
                ROS_INFO_STREAM("Failed to send request");
            }else{
                if(srv.response.hasTask == false){
                    ROS_INFO_STREAM("No available task");
                }else{
                    ROS_INFO_STREAM("Get a task");
                    break;
                }
            }
            ros::Duration(5).sleep();
        }

    }

    void GoToSleep(ros::Time wake_up){
        ROS_INFO_STREAM("\nTime: " <<Util::time_str(ros::Time::now()) << "\nSleep until "<< Util::time_str(wake_up) );
        ros::Time::sleepUntil(wake_up);
        ROS_INFO_STREAM("** Wake up. Time: " <<Util::time_str(ros::Time::now()));  
    }

    // called in a separate thread whenever a new goal is received
    void ExecuteCallback(const robot_navigation::GoToTargetGoalConstPtr &task){
        ROS_INFO_STREAM("Get a task from pool\n"<<*task);
        robot_navigation::GoToTargetResult rs;
        rs.taskId = task->taskId;

        // Wait until task time
        ros::Time now = ros::Time::now();
        if(task->goals[0].header.stamp < now ){
            ROS_INFO_STREAM("Task "<< task->taskId << " is expired");
            _gas.setAborted(rs);
            RequestTask(); // Get a new task
            return;
        }else if (task->goals[0].header.stamp > now - ros::Duration(1)) {
            GoToSleep(now - ros::Duration(1));  // both thread go to sleep
        }
        
        // Start task
        if (task->taskType == "Charging" ){
            StartChargingTask(task->goals[0]);
        }else if (task->taskType == "GatherEnviromentInfo" ){
            StartGatherInviromentTask(task->goals[0]);
        }else if (task->taskType == "ExecuteTask" ){
            StartExecuteTask(task->goals[0]);
        }else{
            ROS_INFO_STREAM("Unknown task");
            _gas.setAborted(rs);
        }

        // wait for task complete
        _movCv.wait(_movLock); 
        
        // Set task succeeded
        rs.isCompleted = true;
        _gas.setSucceeded(rs);   
        ROS_INFO_STREAM("report task result "<<rs);       
        RequestTask(); // Get a new task
    }

     
    // void MoveBaseCompleteCallback(const actionlib::SimpleClientGoalState& state,
    //        const move_base_msgs::MoveBaseResult::ConstPtr& result ){
    //         ROS_INFO_STREAM("Move complete. State "<<state.toString());
    //         if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
    //             State = &RobotController::ProcessTask;
    //         }else{
    //             _rs.isCompleted = false;
    //             _movCv.notify_all();
    //         }
    // }

    void StartChargingTask(geometry_msgs::PoseStamped cs){
        ROS_INFO_STREAM(" Start charging task");
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = cs;
        _mbc.sendGoal(goal,
                boost::bind(&RobotController::WhenChargingTaskComplete,this, _1, _2),
                actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
                boost::bind(&RobotController::MoveBasePositionFeedback,this, _1)
        ); 
    }

    void WhenChargingTaskComplete(const actionlib::SimpleClientGoalState& state,
           const move_base_msgs::MoveBaseResult::ConstPtr& result ){
             ROS_INFO_STREAM("charging 5s....");
            ros::Duration(5).sleep(); // charging for 5sec
            _battery = 100;
            _movCv.notify_all();
    }
 
    void StartGatherInviromentTask(geometry_msgs::PoseStamped door){
        ROS_INFO_STREAM(" Start gather enviroment");
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = door;
        _mbc.sendGoal(goal,
                boost::bind(&RobotController::WhenGatherInviromentTaskComplete,this, _1, _2),
                actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
                boost::bind(&RobotController::MoveBasePositionFeedback,this, _1)
        ); 
    }
    
    void WhenGatherInviromentTaskComplete(const actionlib::SimpleClientGoalState& state,
           const move_base_msgs::MoveBaseResult::ConstPtr& result ){
        ROS_INFO_STREAM("Gather rnviroment finished");
        _movCv.notify_all();
    }

    void StartExecuteTask(geometry_msgs::PoseStamped point){
        ROS_INFO_STREAM(" Start execute task");
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = point;
        _mbc.sendGoal(goal,
                boost::bind(&RobotController::WhenExecuteTaskComplete,this, _1, _2),
                actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
                boost::bind(&RobotController::MoveBasePositionFeedback,this, _1)
        ); 
    }

    void WhenExecuteTaskComplete(const actionlib::SimpleClientGoalState& state,
           const move_base_msgs::MoveBaseResult::ConstPtr& result ){
        ROS_INFO_STREAM("Execute task finished");
        _movCv.notify_all();
    }

    // void ProcessTask(){
    //     if(_taskType == "Charging"){
    //         ROS_INFO_STREAM("charging 5s....");
    //         ros::Duration(5).sleep(); // charging for 5sec
    //         _battery = 100;
    //     }else if(_taskType == "ExecuteTask"){
    //        ROS_INFO_STREAM("doing task 3s....");
    //         ros::Duration(3).sleep(); // charging for 5sec
    //     }
    //     _rs.isCompleted = true;
    //     _movCv.notify_all();
    // }

    void ListenSensorCallback(const robot_navigation::sensor_data::ConstPtr& message){

        double distance = sqrt(pow(_cp.pose.position.x - message->pose.x,2) + pow(_cp.pose.position.y - message->pose.y,2));

        if(distance <= SENSOR_RANGE){
            std::string status = message->doorStatus?"open":"closed";
            ROS_INFO_STREAM( "Distance "<< distance<<" room " << message->id <<" door "<<status<<" position ("<<message->pose.x<<", "<<message->pose.y<<", "<<message->pose.z<<")");
            _fb.doorId = message->id;
            _fb.measureTime = message->stamp;
            _fb.doorStatus = message->doorStatus;
            _fb.robotId = _robotId;
            _gas.publishFeedback(_fb);
        }
    }

        
    void MoveBasePositionFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){
            double distance = sqrt(pow((feedback->base_position.pose.position.x - _cp.pose.position.x),2) + 
                                    pow((feedback->base_position.pose.position.y - _cp.pose.position.y),2));
            _cp = feedback->base_position;

            double angle = 2 * acos(feedback->base_position.pose.orientation.w);
            _battery=  _battery - 0.01 * distance - 0.001 * angle;
            // ROS_INFO_STREAM("angle "<<angle << "distance" << distance << "battery_level"<<_battery);
     }
             
private:
    
    // Note handle
    ros::NodeHandle _nh;
    
    // Publisher
    ros::Publisher _mbp;

    // Subscriber 
    ros::Subscriber _ps;
    ros::Subscriber _ss;

    // client
    ros::ServiceClient _tc;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> _mbc;
    actionlib::SimpleActionServer<robot_navigation::GoToTargetAction> _gas;
    
    robot_navigation::GoToTargetFeedback _fb;
    

    geometry_msgs::PoseStamped _cp;
    
    // int _taskId;
    // int _targetId;
    // string _taskType; 

    int _robotId;
    double _battery;

    boost::mutex _mtx;
    boost::condition_variable _movCv;
    boost::unique_lock<boost::mutex> _movLock;
};

int main(int argc, char **argv){
	ROS_INFO("Main function start");
        ros::init(argc,argv,"robot_controller"); // Initializes Node Name
        
    ROS_INFO("RobotController run");
    RobotController rc(0);

    ros::spin(); 

        return 0;
}
