#include "sql_client.h"
#include "util.h"
#include "ros/ros.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"task_generator");
    SQLClient sc("task_generator","pass");
    ros::NodeHandle nh;
       
    Task t;
    t.task_type = "ExecuteTask";
    t.priority = 4;
    t.goal.pose.position.x = 4.38077210276;
    t.goal.pose.position.y =  9.34650744461;
    t.goal.pose.orientation.z = 0.721488227349;
    t.goal.pose.orientation.w = 0.692426702112;
    t.goal.header.stamp = ros::Time::now()+ros::Duration(20);
    t.goal.header.frame_id = "map";
    t.target_id = sc.InsertATargetAssignId(t.goal);
    sc.InsertATaskAssignId(t);  
    
    ROS_INFO_STREAM("Created a tasks");

}