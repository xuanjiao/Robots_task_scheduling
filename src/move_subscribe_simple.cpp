#include "ros/ros.h"
#include "robot_navigation/sensor_data.h"
#include <cmath>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::Pose current_pos;
const double sensor_range = 3;

void print_pose(const geometry_msgs::Pose p){
    current_pos = p;
    ROS_INFO("my position (%.2f,%.2f,%.2f), orientation(%.2f,%.2f,%.2f,%.2f)",
                p.position.x,p.position.y,p.position.z,
                p.orientation.x,p.orientation.y,p.orientation.z,
                p.orientation.w);
}

void pos_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message){
    print_pose(message->pose.pose);
}

void sensor_callback(const robot_navigation::sensor_data::ConstPtr& message){
    std::string status = message->door_status?"open":"closed";
        
    double distance = sqrt(pow(current_pos.position.x - message->pose.x,2) + pow(current_pos.position.y - message->pose.y,2));

    if(distance <= sensor_range){
        ROS_INFO_STREAM( "Distance "<< distance<<" room " << message->id <<" door "<<status<<" position ("<<message->pose.x<<", "<<message->pose.y<<", "<<message->pose.z<<")");
    }
}

int main(int argc, char **argv){
    ros::init(argc,argv,"move_base_simple"); // Initializes Node Name

    ros::NodeHandle nh;
    
    std::vector<double> position;
    std::vector<double> orientation;

    nh.getParam("/room_pose_i/position",position);
    nh.getParam("/room_pose_i/orientation",orientation);

    // subscribe to current position
    ros::Subscriber pos_sub = 
        nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",1,pos_callback);
    // subscribe to door sensor node
    ros::Subscriber sensor_sub = 
        nh.subscribe<robot_navigation::sensor_data>("sensor_data",100,sensor_callback);


    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);

    // build message
    geometry_msgs::PoseStamped message;

    message.header.frame_id = "map";
    message.header.stamp = ros::Time::now();

     message.pose.position.x = position[0];
     message.pose.position.y = position[1];
     message.pose.position.z = position[2];

     message.pose.orientation.x = orientation[0];
     message.pose.orientation.y = orientation[1];
     message.pose.orientation.z = orientation[2];
     message.pose.orientation.w = orientation[3];

    ros::Rate loop_rate(10);
      
    while(ros::ok()){
	    pub.publish(message);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
