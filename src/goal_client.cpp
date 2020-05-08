#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv){
    ros::init(argc,argv,"goal_client"); // Initializes Node Name
    // Construct a action client and spin up a thread to service this action's subscription.
    MoveBaseClient ac("move_base", true);

	ROS_INFO("Waiting for the move_base action server to start");

    // Wait for the action server to connect to this client
    ac.waitForServer();    	

    ROS_INFO("Action server started, sending goal");

    move_base_msgs::MoveBaseGoal goal;
    double position[3] = {0.05,-2.05,0};
    // Send a goal to action server to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = position[0];
    goal.target_pose.pose.position.y = position[1];
    goal.target_pose.pose.position.z = position[2];
    goal.target_pose.pose.orientation.w = 1.0;

    ac.sendGoal(goal); 

    // Set action to time limit(30s)
    bool finished_before_time_out = ac.waitForResult(ros::Duration(30.0));

    if(finished_before_time_out){
        actionlib::SimpleClientGoalState state = ac.getState(); 
	    ROS_INFO_STREAM("Action finished. "<<state.toString());
        
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO_STREAM(" Go to point("<<
     	    position[0]<<","<<position[1]<<","<<position[2]<<") orientation w = "<< goal.target_pose.pose.orientation.w);
        }

    }else{
	    ROS_INFO("Action did not finished before time out");   
    }
    return 0;
}
