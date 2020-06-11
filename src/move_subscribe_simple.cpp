#include "ros/ros.h"
#include "robot_navigation/sensor_data.h"
#include "robot_navigation/make_task.h"
#include <cmath>
#include <sstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <string> 
#include "util.h"
#include "time_transfer.h"

#define SENSOR_RANGE 1

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
 
class Demo{

public:
    Demo():move_base_client("move_base", true){
        battery_level = 100;
        ROS_INFO_STREAM("Current Office time: "<<TimeTransfer::convert_to_office_time_string(ros::Time::now()));
	    ros::Duration(1).sleep();
        ROS_INFO_STREAM("Current Office time: "<<TimeTransfer::convert_to_office_time_string(ros::Time::now()));

        // subscribe to door sensor node
        sensor_sub = 
            nh.subscribe<robot_navigation::sensor_data>("sensor_data",100,&Demo::sensor_callback,this);      
        task_client = nh.serviceClient<robot_navigation::make_task>("make_task");
        move_base_client.waitForServer();
        request_current_pose();
        
        // request a best task from centralized pool and do this task
        talk_to_centralized_pool();

        ros::spin();
    }

    void request_current_pose(){
        // try to get its current location
        boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> sharedPtr =
             ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",nh);
        if(sharedPtr == NULL){
           ROS_DEBUG("Failed to get current position");
           return;
        }
        
        current_pos = sharedPtr->pose.pose;
        ROS_INFO_STREAM("Robot get current position "<<Util::pose_str(current_pos));       
    }

    void talk_to_centralized_pool(){
        // request a best task
        robot_navigation::make_task srv;
        srv.request.battery_level = battery_level;
        srv.request.pose = current_pos;
        srv.request.last_task = current_task;
        srv.request.last_task.is_completed = true;
        srv.request.last_task.door_status = current_task.door_status;
        ROS_INFO_STREAM("send task request. start "<<Util::pose_str(srv.request.pose));

        if(!task_client.call(srv)){
            ROS_INFO_STREAM("Failed to send request");
            return;
        }
        current_task = srv.response.best_task;
        ROS_INFO_STREAM("receive response best task  "<<" time "<<TimeTransfer::convert_to_office_time_string(current_task.goal.header.stamp)
            <<"Position "<<Util::pose_str(current_task.goal.pose));
        run_task();
    }

    void run_task(){
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = current_task.goal;
        ROS_INFO_STREAM("Current office time: "<<TimeTransfer::convert_to_office_time_string(ros::Time::now())<<
                        "\nSimulation time: " <<ros::Time::now().sec <<
                        "\nSleep until "<< TimeTransfer::convert_to_office_time_string(goal.target_pose.header.stamp) <<
                        "\nSimulation time: " <<goal.target_pose.header.stamp.sec
        );
            
        ros::Time::sleepUntil(goal.target_pose.header.stamp);
        ROS_INFO_STREAM("** Wake up. Current office time: "<<TimeTransfer::convert_to_office_time_string(ros::Time::now()));
        
        move_base_client.sendGoal(goal,
                boost::bind(&Demo::move_complete_callback,this, _1, _2),
                MoveBaseClient::SimpleActiveCallback(),
                boost::bind(&Demo::move_position_feedback,this, _1));        
    }

    void move_position_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){
            current_pos = feedback->base_position.pose;
            battery_level-=0.01;
            // ROS_INFO_STREAM("Current position: "<<Util::pose_str(current_pos));
     }
     
    void move_complete_callback(const actionlib::SimpleClientGoalState& state,
           const move_base_msgs::MoveBaseResult::ConstPtr& result ){
            if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
                    ROS_INFO("Goal reached!");

                    // complete one task, request next one
                    talk_to_centralized_pool();
            }else
                    ROS_INFO("Goal failed");
    }


    void sensor_callback(const robot_navigation::sensor_data::ConstPtr& message){

        double distance = sqrt(pow(current_pos.position.x - message->pose.x,2) + pow(current_pos.position.y - message->pose.y,2));

        if(distance <= SENSOR_RANGE){
                std::string status = message->door_status?"open":"closed";
                current_task.door_status = message->door_status;
            ROS_INFO_STREAM( "Distance "<< distance<<" room " << message->id <<" door "<<status<<" position ("<<message->pose.x<<", "<<message->pose.y<<", "<<message->pose.z<<")");
        }
    }

private:
    
    // Note handle
    ros::NodeHandle nh;
    
    // Publisher
    ros::Publisher move_base_pub;

    // Subscriber 
    ros::Subscriber pos_sub;
    ros::Subscriber sensor_sub;

    // client
    ros::ServiceClient task_client;
    MoveBaseClient move_base_client;

    geometry_msgs::Pose current_pos;
    robot_navigation::task_info current_task;
    double battery_level;
};

int main(int argc, char **argv){
	ROS_INFO("Main function start");
        ros::init(argc,argv,"move_base_simple"); // Initializes Node Name
        
    ROS_INFO("Demo run");
        Demo demo;

        ros::spin();      

        return 0;
}
