#include "ros/ros.h"
#include "robot_navigation/sensor_data.h"
#include <cmath>
#include <sstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <string> 

#define SENSOR_RANGE 3

class Demo{

public:
    Demo(){

	ROS_INFO("constructor run");
        nh.getParam("/room_pose_i/position",target_pose_position);
        nh.getParam("/room_pose_i/orientation",target_pose_orientation);

        // subscribe to current position
        pos_sub = 
            nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",1,&Demo::pos_callback,this);
        // subscribe to door sensor node
        sensor_sub = 
            nh.subscribe<robot_navigation::sensor_data>("sensor_data",100,&Demo::sensor_callback,this);
        plan_sub = 
            nh.subscribe<nav_msgs::Path>("move_base/NavfnROS/plan",1,&Demo::plan_callback,this);

        pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);

        // build message
        geometry_msgs::PoseStamped message;

        message.header.frame_id = "map";
        message.header.stamp = ros::Time::now();

        message.pose.position.x = target_pose_position[0];
        message.pose.position.y = target_pose_position[1];
        message.pose.position.z = target_pose_position[2];

        message.pose.orientation.x = target_pose_orientation[0];
        message.pose.orientation.y = target_pose_orientation[1];
        message.pose.orientation.z = target_pose_orientation[2];
        message.pose.orientation.w = target_pose_orientation[3];

        pub.publish(message);

        ros::Rate loop_rate(2); // 2hz

        while(ros::ok()){
            pub.publish(message);
            ros::spinOnce();
            loop_rate.sleep();
        }
        
    }
    
    void pos_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message){
        
        ROS_INFO_STREAM("Time"<<time_str(ros::Time::now())<<" Position"<<pose_str(message->pose.pose));
    }

    void sensor_callback(const robot_navigation::sensor_data::ConstPtr& message){
        std::string status = message->door_status?"open":"closed";
            
        double distance = sqrt(pow(current_pos.position.x - message->pose.x,2) + pow(current_pos.position.y - message->pose.y,2));

        if(distance <= SENSOR_RANGE){
            ROS_INFO_STREAM( "Distance "<< distance<<" room " << message->id <<" door "<<status<<" position ("<<message->pose.x<<", "<<message->pose.y<<", "<<message->pose.z<<")");
        }
    }

    void plan_callback(const nav_msgs::Path::ConstPtr& messgae){
        
        double distance = 0.0;
        int size = messgae->poses.size();
        
        for(int i = 1; i < size;i++){
            distance += sqrt(pow((messgae->poses[i].pose.position.x - messgae->poses[i-1].pose.position.x),2) + 
                                pow((messgae->poses[i].pose.position.y - messgae->poses[i-1].pose.position.y),2));
        }
 
        ROS_INFO_STREAM("Distance to goal: "<<distance);
    }

    std::string pose_str(const geometry_msgs::Pose p){
        current_pos = p;
        std::stringstream ss;
        ss.precision(2);
        ss << "("<<p.position.x <<"," <<p.position.y <<","<< p.position.z<<")";
        return ss.str();
    }

    std::string time_str(ros::Time time){
        const int output_size = 100;
        char output[output_size];
        std::string format = "%Y-%m-%d %H:%M:%S"; // time format
        std::time_t  raw_time = static_cast<time_t>(time.sec); // convert ros time to time_t
        struct tm* time_info = localtime(&raw_time);
        std::strftime(output,output_size,format.c_str(),time_info);
        return "Time: " + std::string(output);
    }

private:
    
    // Note handle
    ros::NodeHandle nh;
    
    // Publisher
    ros::Publisher pub;

    // Subscriber 
    ros::Subscriber pos_sub;
    ros::Subscriber sensor_sub;
    ros::Subscriber plan_sub;

    geometry_msgs::Pose current_pos;
        
    std::vector<double> target_pose_position;
    std::vector<double> target_pose_orientation;
    

};

int main(int argc, char **argv){
	ROS_INFO("Main function start");
        ros::init(argc,argv,"move_base_simple"); // Initializes Node Name
        
    ROS_INFO("Demo run");
        Demo demo;

        ros::spin();      

        return 0;
}
