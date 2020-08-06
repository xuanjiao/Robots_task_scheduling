#include "sql_client.h"
#include "util.h"
#include "ros/ros.h"
#include <iostream>

using namespace std;

int main(int argc, char** argv){
    ros::init(argc,argv,"task_generator");
    SQLClient sc("task_generator","pass");
    ros::NodeHandle nh;
   
    ros::Rate loop(2);
    while(ros::ok()){
            TaskInTable t;
            t.taskType = "ExecuteTask";

            cout << "Please enter priority(2-4) , x , y , dependency"<<endl;

            cin >> t.priority;
            cin >> t.goal.pose.position.x;
            cin >> t.goal.pose.position.y;
            cin >> t.dependency;
            t.goal.pose.orientation.z = 0.1;
            t.goal.header.stamp = ros::Time::now()+ros::Duration(60);
            t.goal.header.frame_id = "map";
    
            t.targetId = sc.InsertATargetAssignId(t.goal,"Point");
            int taskId = sc.InsertATaskAssignId(t);  
            
            // ROS_INFO_STREAM("Created a task. \n"<<t.goal<<" target id = "<<t.targetId<< " task id ="<<taskId);
            // ROS_INFO_STREAM("Hello");
            cout << "Create task "<<taskId<<endl;
            // ros::spinOnce();
            // loop.sleep();
    }   

    // t.goal.pose.position.x = 4.38077210276;
    // t.goal.pose.position.y =  9.34650744461;
    // t.goal.pose.orientation.z = 0.721488227349;
    // t.goal.pose.orientation.w = 0.692426702112;

    // t.goal.pose.position.x = -19.4279;
    // t.goal.pose.position.y =  9.00543;
    // t.goal.pose.orientation.z = 0.673663;
    // t.goal.pose.orientation.w = 0.739039;


   
}