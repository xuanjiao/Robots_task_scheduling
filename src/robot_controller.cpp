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

enum State{
    SLEEP = 0,
    REQUEST = 1,
    GETTASK = 2,
    MOVING = 3
};

class RobotController{

public:
    RobotController():
    _mbc("move_base", true),
    _gas(_nh,"GoToTargetAction",boost::bind(&RobotController::ExecuteCallback,this,_1),false),
    _movLock(_mtx)
    {
        _robotId = 1;
        _battery = 100;

        _ss = _nh.subscribe<robot_navigation::sensor_data>("sensor_data",100,&RobotController::ListenSensorCallback,this);      
        _tc = _nh.serviceClient<robot_navigation::GetATask>("GetATask");
       
        _gas.start();  // start receive goal from server
        State = &RobotController::RequestCurrentPosition;
    }


    void RequestCurrentPosition(){
        _mbc.waitForServer();

        // try to get its current location
        boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> sharedPtr =
             ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",_nh);
        if(sharedPtr == NULL){
           ROS_DEBUG("Failed to get current position");
           return;
        }     
        _cp.pose = sharedPtr->pose.pose;
        ROS_INFO_STREAM("Robot get current position "<<Util::pose_str(_cp.pose)); 
        
        // request a best task from centralized pool and do this task
        State = &RobotController::RequestTask;      
    }

    void GoToSleep(){
        ros::Time wake_up = _tp.header.stamp - ros::Duration(0.1);
        ROS_INFO_STREAM(
                        "\nTime: " <<Util::time_str(ros::Time::now()) <<
                        "\nSleep until "<< Util::time_str(wake_up)
        );
        ros::Time::sleepUntil(wake_up - ros::Duration(0.1));                
        ROS_INFO_STREAM("** Wake up. Time: " <<Util::time_str(ros::Time::now()));
        
        State = &RobotController::SendGoalToMoveBase;
    }

// called in a separate thread whenever a new goal is received
void ExecuteCallback(const robot_navigation::GoToTargetGoalConstPtr &goal){
    ROS_INFO_STREAM("Get a goal from pool\n"<<*goal);
        _tp = goal->goal;
        _targetId = goal->target_id;
        _taskId = goal->task_id;
        _taskType = goal->task_type;

        if(_tp.header.stamp < ros::Time::now()){
            ROS_INFO_STREAM("Task is expired");
            _gas.setAborted(_rs);
            State = &RobotController::RequestCurrentPosition;
            return;
        // }
        // else if (_tp.header.stamp > ros::Time::now()){
        //     State = &RobotController::GoToSleep;  // main thread go to sleep
        }else{
             State = &RobotController::SendGoalToMoveBase; // main thread send goal
            _movCv.wait(_movLock); // Wait until robot arrive goal
            if(_rs.isCompleted){
                _rs.task_id = _taskId;
                _gas.setSucceeded(_rs);
            }else{
                ROS_INFO_STREAM("Failed to go to target");
                _gas.setAborted(_rs);
            }
            ROS_INFO_STREAM("report task result "<<_rs);       
            State = &RobotController::RequestTask;
        }
        
    }

    // request a task from centralized pool
    void RequestTask(){
        robot_navigation::GetATask srv;
        srv.request.batteryLevel=  _battery;
        srv.request.lastTaskId = (int8_t)_taskId;
        srv.request.pose = _cp.pose;
        if(!_tc.call(srv)){
            ROS_INFO_STREAM("Failed to send request");
            State = &RobotController::RequestTask;
        }else{
            ROS_INFO_STREAM("receive response: has task? "<<srv.response.hasTask?"Yes":"No");
        }
        ros::spinOnce(); // wait for goal client 
    }


    void SendGoalToMoveBase(){
        ROS_INFO_STREAM("Send goal to move base");
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = _tp;
        _mbc.sendGoal(goal,
                boost::bind(&RobotController::MoveBaseCompleteCallback,this, _1, _2),
                actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
                boost::bind(&RobotController::MoveBasePositionFeedback,this, _1)
        ); 

         State = &RobotController::StateMoving;
    }

    void StateMoving(){
        // ROS_INFO("Task running");
         
    }

    void MoveBasePositionFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){
            double distance = sqrt(pow((feedback->base_position.pose.position.x - _cp.pose.position.x),2) + 
                                    pow((feedback->base_position.pose.position.y - _cp.pose.position.y),2));
            _cp = feedback->base_position;

            double angle = 2 * acos(feedback->base_position.pose.orientation.w);
            _battery=  _battery - 0.01 * distance - 0.001 * angle;
            // ROS_INFO_STREAM("angle "<<angle << "distance" << distance << "battery_level"<<_battery);
     }
     
    void MoveBaseCompleteCallback(const actionlib::SimpleClientGoalState& state,
           const move_base_msgs::MoveBaseResult::ConstPtr& result ){
            ROS_INFO_STREAM("Move complete. State "<<state.toString());
            if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
                State = &RobotController::ProcessTask;
            }else{
                _rs.isCompleted = false;
                _movCv.notify_all();
            }
    }

    void ProcessTask(){
        if(_taskType == "Charging"){
            ROS_INFO_STREAM("charging 5s....");
            ros::Duration(5).sleep(); // charging for 5sec
            _battery = 100;
        }else if(_taskType == "ExecuteTask"){
           ROS_INFO_STREAM("doing task 3s....");
            ros::Duration(3).sleep(); // charging for 5sec
        }
        _rs.isCompleted = true;
        _movCv.notify_all();
    }

    void ListenSensorCallback(const robot_navigation::sensor_data::ConstPtr& message){

        double distance = sqrt(pow(_cp.pose.position.x - message->pose.x,2) + pow(_cp.pose.position.y - message->pose.y,2));

        if(distance <= SENSOR_RANGE){
            std::string status = message->door_status?"open":"closed";
            ROS_INFO_STREAM( "Distance "<< distance<<" room " << message->id <<" door "<<status<<" position ("<<message->pose.x<<", "<<message->pose.y<<", "<<message->pose.z<<")");
            _fb.door_id = message->id;
            _fb.m_time = message->stamp;
            _fb.door_status = message->door_status;
            _fb.robot_id = _robotId;
            _gas.publishFeedback(_fb);
        }
    }


void (RobotController::*State)();               
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
    robot_navigation::GoToTargetResult _rs;

    geometry_msgs::PoseStamped _cp;
    geometry_msgs::PoseStamped _tp;
    
    int _taskId;
    int _targetId;
    string _taskType; 

    int _robotId;
    double _battery;

    boost::mutex _mtx;
    boost::condition_variable _movCv;
    boost::unique_lock<boost::mutex> _movLock;
};

int main(int argc, char **argv){
	ROS_INFO("Main function start");
        ros::init(argc,argv,"move_base_simple"); // Initializes Node Name
        
    ROS_INFO("RobotController run");
        RobotController* rc = new RobotController();

        while(ros::ok()){
            ros::spinOnce();
            (rc->*(rc->State))();
            // loop.sleep();
            ros::Duration(3).sleep();
        }   

        return 0;
}
