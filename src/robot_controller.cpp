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
    RUN = 3
};

class RobotController{

public:
    RobotController():
    _mbc("move_base", true),
    _gas(_nh,"GoToTargetAction",boost::bind(&RobotController::execute_callback,this,_1),false),
    _lk(_mtx)
    {
        _robotId = 1;
        _battery = 100;
        
	    // ros::Duration(1).sleep();
        

        // subscribe to door sensor node
        _ss = _nh.subscribe<robot_navigation::sensor_data>("sensor_data",100,&RobotController::sensor_callback,this);      
        // _tc = nh.serviceClient<robot_navigation::make_task>("make_task");
        _tc = _nh.serviceClient<robot_navigation::GetATask>("GetATask");
        
        // start receive goal from server
        _gas.start();

        State = &RobotController::request_current_pose;
    }


    void request_current_pose(){
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
        State = &RobotController::request_task;      
    }

    void go_to_sleep(){
        ros::Time wake_up = _tp.header.stamp - ros::Duration(0.1);
        if( wake_up < ros::Time::now()){
            ROS_INFO_STREAM("Task is expired");
            // next_mode(State::REQUEST,false);
            return;
        }
        ROS_INFO_STREAM(
                        "\nSimulation time: " <<ros::Time::now().sec <<
                        "\nSleep until "<< Util::time_str(wake_up) <<
                        "\nSimulation time: " <<wake_up.sec
        );
        ros::Time::sleepUntil(wake_up - ros::Duration(0.1));                
        ROS_INFO_STREAM("** Wake up.Simulation time: " <<ros::Time::now().sec );
        State = &RobotController::start_moving;
        // next_mode(State::RUN,true);
    }

// called in a separate thread whenever a new goal is received
void execute_callback(const robot_navigation::GoToTargetGoalConstPtr &goal){
    ROS_INFO_STREAM("Get a goal from pool\n"<<*goal);
        _tp = goal->goal;
        _targetId = goal->target_id;
        _taskId = goal->task_id;
        _taskType = goal->task_type;

        if(_tp.header.stamp < ros::Time::now()){
            ROS_INFO_STREAM("Task is expired");
            State = &RobotController::request_task;
            return;
        }
        State = &RobotController::go_to_sleep;
        
         _cv.wait(_lk);

        ros::Rate loop(5);
        if(_rs.isCompleted){
                _rs.task_id = _taskId;
                _gas.setSucceeded(_rs);
            }else{
                ROS_INFO_STREAM("Failed to go to target");
                _gas.setAborted(_rs);
        }
        ROS_INFO_STREAM("report task result "<<_rs);       
        State = &RobotController::request_task;
    }

    // void talk_to_centralized_pool(bool is_complete){
    //     // request a best task
    //     robot_navigation::make_task srv;
    //     srv.request.battery_level = battery_level;
    //     srv.request.pose = current_pos;
    //     srv.request.last_task = current_task;
    //     srv.request.last_task.m_time = ros::Time::now();
    //     srv.request.last_task.is_completed = is_complete;

    //     srv.request.last_task.door_status = current_task.door_status;
    //     ROS_INFO_STREAM("send task request. Robot position: "<<Util::pose_str(srv.request.pose));

    //     if(!_tc.call(srv)){
    //         ROS_INFO_STREAM("Failed to send request");
    //         return;
    //     }
    //     current_task = srv.response.best_task;
    //     ROS_INFO_STREAM("receive response\n"<<current_task);
        
    //     // next_mode(State::SLEEP,true);
    // }

    // request a task from centralized pool
    void request_task(){
        robot_navigation::GetATask srv;
        srv.request.battery_level =  _battery;
        srv.request.last_task_id = _taskId;
        srv.request.pose = _cp.pose;
        if(!_tc.call(srv)){
            ROS_INFO_STREAM("Failed to send request");
            State = &RobotController::request_task;
        }  

        ros::spinOnce(); // wait for goal client

        ROS_INFO_STREAM("receive response: has task? "<<srv.response.has_task);
        
        //  state = &RobotController::request_task;
        // next_mode(State::SLEEP,true);
       
    }


    void start_moving(){
        ROS_INFO_STREAM("send target to move base");
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = _tp;
        _mbc.sendGoal(goal,
                boost::bind(&RobotController::move_complete_callback,this, _1, _2),
                actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
                boost::bind(&RobotController::move_position_feedback,this, _1)
        ); 

         State = &RobotController::task_running;
    }

    void task_running(){
        // ROS_INFO("Task running");
    }

    void move_position_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){
            double distance = sqrt(pow((feedback->base_position.pose.position.x - _cp.pose.position.x),2) + 
                                    pow((feedback->base_position.pose.position.y - _cp.pose.position.y),2));
            _cp = feedback->base_position;

            double angle = 2 * acos(feedback->base_position.pose.orientation.w);
            _battery=  _battery - 0.01 * distance - 0.001 * angle;
            // ROS_INFO_STREAM("angle "<<angle << "distance" << distance << "battery_level"<<_battery);
     }
     
    void move_complete_callback(const actionlib::SimpleClientGoalState& state,
           const move_base_msgs::MoveBaseResult::ConstPtr& result ){
            ROS_INFO_STREAM("Move complete. State "<<state.toString());
            if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
                _rs.isCompleted = true;
            }else{
                _rs.isCompleted = false;
            }
            _cv.notify_one();
    }

    void sensor_callback(const robot_navigation::sensor_data::ConstPtr& message){

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
    boost::condition_variable _cv;
    boost::unique_lock<boost::mutex> _lk;
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
