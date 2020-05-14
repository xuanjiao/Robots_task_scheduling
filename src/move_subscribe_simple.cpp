#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

void print_pose(const geometry_msgs::Pose p){
 ROS_INFO("position (%.2f,%.2f,%.2f), orientation(%.2f,%.2f,%.2f,%.2f)",
                p.position.x,p.position.y,p.position.z,
                p.orientation.x,p.orientation.y,p.orientation.z,
                p.orientation.w);
}

void callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message){
    print_pose(message->pose.pose);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"move_base_simple"); // Initializes Node Name

    ros::NodeHandle nh;
    
    std::vector<double> position;
    std::vector<double> orientation;

    nh.getParam("/room_pose_i/position",position);
    nh.getParam("/room_pose_i/orientation",orientation);


    ros::Subscriber sub = 
        nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",1,callback);

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);

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
