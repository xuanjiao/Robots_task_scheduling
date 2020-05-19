#include "ros/ros.h"
#include "robot_navigation/make_task.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <vector>
#include <string>

#define ROOM_NUM 3

typedef struct {
    int priority;
    int task_id;
    char room_id;
    geometry_msgs::PoseStamped goal; // distination and timestamp
}EnterRoomTask;

class CentralizedPool{

public:
    CentralizedPool(){

        load_room_position();

        //nh.getParam("/room_pose_i/position",target_pose_position);
        //nh.getParam("/room_pose_i/orientation",target_pose_orientation);

        // Create available tasks


        // Create a server, usiing make_task.srv file. The service name is make_task
        // task_server = nh.advertiseService<robot_navigation::make_task>("make_task",choose_best_task);
    
        // Create a client for service "make_plan"
        // plan_client = nh.serviceClient<nav_msgs::GetPlan>("make_plan");

    }

    void load_room_position(){
        
        std::string pose_path,ori_path;
        bool ret = false;
        std::vector<double> temp_pos;
        std::vector<double> temp_ori;
        // load task distination
        for(int i = 0; i < ROOM_NUM ; i++){
        
            pose_path = "/room_pose_" + std::string(1,'a' + i)+"/position";
            ori_path = "/room_pose_" + std::string(1,'a' + i)+"/orientation";
            nh.getParam(pose_path,temp_pos);
            nh.getParam(ori_path,temp_ori);

            geometry_msgs::Pose pose;
            if(temp_pos.size() != 3 || temp_ori.size()!=4){
                ROS_INFO_STREAM("load params failed. position "
                    <<temp_pos.size()<<" orientation "<<temp_ori.size());
                return;
            }
            pose.position.x = temp_pos[0];
            pose.position.y = temp_pos[1];
            pose.position.z = temp_pos[2];

            pose.orientation.x = temp_ori[0];
            pose.orientation.y = temp_ori[1];
            pose.orientation.z = temp_ori[2];
            pose.orientation.w = temp_ori[3];

            room_map.insert(std::pair<char,geometry_msgs::Pose>('a'+i,pose));

        }
        ROS_INFO_STREAM("load "<<room_map.size()<<"positions");
    }

    bool choose_best_task(robot_navigation::make_task::Request &req,robot_navigation::make_task::Response &res){
        
        // request plan with start point and end point

        // Calculate all distances of all plan

        // Choose the one with shortest distance


        // Declare the service
        nav_msgs::GetPlan make_plan_srv;
    
        geometry_msgs::PoseStamped start;
    

    }

    void create_random_tasks(int num,ros::Time start_time){
        
        int cnt = 0;
        long increase_sec;
        int output_size = 100;
        char output[output_size];
        std::string format = "%Y-%m-%d %H:%M:%S"; // time format
        std::time_t  raw_time;
        ros::Time time;
        

        while(cnt < num){
            increase_sec = rand();
            
            // Create a random time after start time 
            time = start_time + ros::Duration(increase_sec);
            
            raw_time = static_cast<time_t>(time.sec); // convert ros time to time_t
            struct tm* time_info = localtime(&raw_time);

            if (time_info->tm_wday > 0 && time_info->tm_wday < 6 && time_info->tm_hour > 9 && time_info->tm_hour < 20) {
			    
                // Create a task on Monday to Friday from 9 am to 20 pm
                EnterRoomTask task;
                task.goal.header.stamp = time;  // set task time
                task.priority = rand()%4 + 1;   // set random task priority 1-5
                task.room_id = rand()%ROOM_NUM + 'A';

                std::strftime(output,output_size,format.c_str(),time_info);

            }
        }
    }
private:
    ros::ServiceServer task_server;

    ros::ServiceClient plan_client;

    std::vector<EnterRoomTask> tasks;

    ros::NodeHandle nh;

    std::map<char,geometry_msgs::Pose> room_map;

};

int main(int argc, char** argv){
    ros::init(argc,argv,"centralized_poor");
    
    CentralizedPool pool;
    

    // Give request value
   // make_plan_srv.request.start(start);
    
}
