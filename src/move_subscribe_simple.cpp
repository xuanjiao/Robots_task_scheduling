#include "ros/ros.h"
#include "robot_navigation/sensor_data.h"
#include "robot_navigation/make_task.h"
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
        // subscribe to current position
        pos_sub = 
            nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",1,&Demo::pos_callback,this);
        // subscribe to door sensor node
        sensor_sub = 
            nh.subscribe<robot_navigation::sensor_data>("sensor_data",100,&Demo::sensor_callback,this);
        // plan_sub = 
        //    nh.subscribe<nav_msgs::Path>("move_base/NavfnROS/plan",1,&Demo::plan_callback,this);
        
        move_base_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);

        task_client = nh.serviceClient<robot_navigation::make_task>("make_task");

      // run_task();

        ros::Rate loop_rate(2); // 2hz

        while(ros::ok()){
            //pub.publish(message);
             // request a best task from centralized pool and do this task
            
            ros::spinOnce();
            loop_rate.sleep();
        }
        
    }

    void run_task(){
        // request a best task
        robot_navigation::make_task srv;
        srv.request.battery_level = 100;
         srv.request.pose = current_pos;
        //srv.request.pose.position.x = 100;
        
        ROS_INFO_STREAM("send request "<<srv.request);

        if(!task_client.call(srv)){
            ROS_INFO_STREAM("Failed to send request");
            return;
        }

        ROS_INFO_STREAM("receive response "<<srv.response);
        
        // // build message
        // geometry_msgs::PoseStamped message;
        // message.header.frame_id = "map";
        // message.header.stamp = ros::Time::now();
        // message.pose = srv.response.best_task;

        move_base_pub.publish(srv.response.best_task);
        
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
    ros::Publisher move_base_pub;

    // Subscriber 
    ros::Subscriber pos_sub;
    ros::Subscriber sensor_sub;
    ros::Subscriber plan_sub;

    // client
    ros::ServiceClient task_client;
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
