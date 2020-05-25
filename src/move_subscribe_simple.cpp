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

#define SENSOR_RANGE 1

    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Demo{

public:
    Demo():move_base_client("move_base", true){
        battery_level = 100;

     // subscribe to door sensor node
        sensor_sub = 
            nh.subscribe<robot_navigation::sensor_data>("sensor_data",100,&Demo::sensor_callback,this);
        
        task_client = nh.serviceClient<robot_navigation::make_task>("make_task");
 
        move_base_client.waitForServer();

        request_current_pose();
        
        // request a best task from centralized pool and do this task
        run_task();

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
        ROS_INFO_STREAM("Robot get current position "<<pose_str(current_pos));       
    }

    void run_task(){
        // request a best task
        robot_navigation::make_task srv;
        srv.request.battery_level = battery_level;
        srv.request.pose = current_pos;
        
        ROS_INFO_STREAM("send task request. start "<<pose_str(srv.request.pose));

        if(!task_client.call(srv)){
            ROS_INFO_STREAM("Failed to send request");
            return;
        }

        ROS_INFO_STREAM("receive response best task  "<<" time "<<time_str(srv.response.best_task.header.stamp)
            <<"Position "<<pose_str(srv.response.best_task.pose));
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = srv.response.best_task;
        ROS_INFO_STREAM("Current time: "<<time_str(ros::Time::now())<<" sleep until "<<
            time_str(goal.target_pose.header.stamp));
            
        ros::Time::sleepUntil(goal.target_pose.header.stamp);
        ROS_INFO_STREAM("Current time: "<<time_str(ros::Time::now()));
        
        move_base_client.sendGoal(goal,
                boost::bind(&Demo::move_complete_callback,this, _1, _2),
                MoveBaseClient::SimpleActiveCallback(),
             // MoveBaseClient::SimpleFeedbackCallback());
                boost::bind(&Demo::move_position_feedback,this, _1));
        // move_base_pub.publish(srv.response.best_task);
        
    }

    void move_position_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){
            current_pos = feedback->base_position.pose;
            battery_level-=0.1;
            // ROS_INFO_STREAM("Current position: "<<pose_str(current_pos));
     }
     
    void move_complete_callback(const actionlib::SimpleClientGoalState& state,
           const move_base_msgs::MoveBaseResult::ConstPtr& result ){
            if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
                    ROS_INFO("Goal reached!");

                    // complete one task, request next one
                    run_task();
            }else
                    ROS_INFO("Goal failed");
    }


    void sensor_callback(const robot_navigation::sensor_data::ConstPtr& message){
        std::string status = message->door_status?"open":"closed";
            
        double distance = sqrt(pow(current_pos.position.x - message->pose.x,2) + pow(current_pos.position.y - message->pose.y,2));

        if(distance <= SENSOR_RANGE){
            ROS_INFO_STREAM( "Distance "<< distance<<" room " << message->id <<" door "<<status<<" position ("<<message->pose.x<<", "<<message->pose.y<<", "<<message->pose.z<<")");
        }
    }

    // void plan_callback(const nav_msgs::Path::ConstPtr& messgae){
        
    //     double distance = 0.0;
    //     int size = messgae->poses.size();
        
    //     for(int i = 1; i < size;i++){
    //         distance += sqrt(pow((messgae->poses[i].pose.position.x - messgae->poses[i-1].pose.position.x),2) + 
    //                             pow((messgae->poses[i].pose.position.y - messgae->poses[i-1].pose.position.y),2));
    //     }

    //     ROS_INFO_STREAM("Distance to goal: "<<distance);
    // }

    std::string pose_str(const geometry_msgs::Pose p){
        std::stringstream ss;
        ss.precision(3);
        ss << "("<<p.position.x <<", " <<p.position.y <<", "<< p.position.z<<")";
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
    MoveBaseClient move_base_client;

    
    geometry_msgs::Pose current_pos;
  
    // flag
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
