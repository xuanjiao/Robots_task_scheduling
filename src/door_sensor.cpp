#include "ros/ros.h"
#include "robot_navigation/sensor_data.h"
#include "stdio.h"
#include <sstream>
#include "geometry_msgs/Point.h"

int main(int argc, char** argv){
    std::vector<double> position;
    char room_id=argv[1][0];

    // for(int i = 0; i < argc;i++){
    //     ROS_INFO_STREAM("params: "<<argv[i]);
    // }

    if( !((room_id >='A' && room_id <='Z')|| (room_id >='a' && room_id < 'z'))){
        ROS_INFO("Please enter a room id (a-z)");
        return 1;
    }
    // convert big letter to small letter
    if(room_id >='A' && room_id <='Z'){
        room_id += 'a'-'A';
    }

    ros::init(argc,argv,"door_sensor");
    ros::NodeHandle nh;

    // get position of this room
    std::stringstream stream;
    stream << "/room_pose_" << room_id << "/position";
    nh.getParam(stream.str(),position);
    ROS_INFO_STREAM(stream.str());

    // publish sensor data
    ros::Publisher pub = nh.advertise<robot_navigation::sensor_data>("sensor_data",100);

    ros::Rate loop_rate(0.5); // period = 2s 
    
    std::string format = "%Y-%m-%d %H:%M:%S";

    robot_navigation::sensor_data msg;

    // add door id and position

    if(position.size())
    {
        msg.pose.x = position[0];
        msg.pose.y = position[1];
        msg.pose.z = position[2];
    }
    
    while(ros::ok()){
        const int output_size = 100;
        char output[output_size];
   
        msg.stamp = ros::Time::now();
        msg.id = room_id;
        
        // construct time string
        std::time_t  raw_time = static_cast<time_t>(msg.stamp.sec); // convert ros time to time_t
        struct tm* time_info = localtime(&raw_time);
        std::strftime(output,output_size,format.c_str(),time_info);
    
        // door open from 12pm to 18pm
        if(time_info->tm_hour >= 12 && time_info->tm_hour <=18){
            msg.door_status = true;
        }else{
            msg.door_status = false;
        }
        std::string status = msg.door_status?"open":"closed";
        
        ROS_INFO_STREAM(output<<" door "<<status<<" position ("<<msg.pose.x<<", "<<msg.pose.y<<", "<<msg.pose.z<<")");
        
        pub.publish(msg);
        loop_rate.sleep();

    }
    
    return 0;
}