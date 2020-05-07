// ROS Default Header File
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>


int main(int argc, char **argv)  // Node Main Function
{
  ros::init(argc, argv, "topic_publisher"); // Initializes Node Name
  ros::NodeHandle nh;  // Node handle declaration for communication with ROS system

  // Declare publisher, create publisher 'pub' 
  // the size of the publisher queue is set to 100.
   ros::Publisher pub = 
   	nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

  // Set the loop period. '10' refers to 10 Hz and the main loop repeats at 0.1 second intervals  
   ros::Rate loop_rate(10);
   geometry_msgs::Twist msg;
    
   int count = 0;     // Variable to be used in message
 
  while (ros::ok())
   {
     msg.linear.x = 0.2;
     msg.angular.z = 0.6;
     ROS_INFO_STREAM("Sending velocity command:"<<" linear="<<msg.linear.x<<" angular="<<msg.angular.z);
    
     pub.publish(msg);  // Publishes 'msg' message
     loop_rate.sleep();   // Goes to sleep according to the loop rate defined above.
     ++count;     // Increase count variable by one
   }
  return 0;
 } 


