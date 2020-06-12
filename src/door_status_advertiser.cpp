#include "ros/ros.h"
#include <signal.h>
#include <stdio.h>
#include <mysql/mysql.h>
#include <sstream>
#include "util.h"
#include "robot_navigation/sensor_data.h"
#include "sql_client.h"
#include "time_transfer.h"
#include "task_process.h"
using namespace std;
class Advertiser{
    public:
    Advertiser():sql_client(){
        ROS_INFO_STREAM("using simulation time "<<ros::Time::isSimTime());
        ROS_INFO_STREAM("using system time "<<ros::Time::isSystemTime());
        ros::Duration(1).sleep();
        ROS_INFO_STREAM("Current Office time "<<TimeTransfer::convert_to_office_time_string(ros::Time::now()));
       
        sql_client.connect_to_database();
        load_room_position();
        pub = nh.advertise<robot_navigation::sensor_data>("sensor_data",100);

        while(ros::ok()){
            publish_door_status();
            ros::spinOnce();
            ros::Duration(10).sleep();
        }
    }

    void load_room_position(){
        if(sql_client.query_rooms_position(room_map)>0){
            ROS_INFO_STREAM("load "<<room_map.size()<< " room positions");
        } else{
            ROS_INFO_STREAM("load room position failed");
            exit(1);
        }
    }

    bool publish_door_status(){
        std::vector<PossibilityTableRow> table_rows;
        std::vector<DoorStatusListRow> list_rows;
        ros::Time now = ros::Time::now();
        if(!sql_client.query_posibility_table_rooms(table_rows,list_rows,TimeTransfer::convert_to_office_time(now))){
 			ROS_INFO_STREAM("No result. Current Office time: "<<TimeTransfer::convert_to_office_time_string(now));
            return false;
        }
        for(DoorStatusListRow row : list_rows){  
            robot_navigation::sensor_data msg;
            msg.stamp = now;
            msg.id = row.room_id; 
            msg.pose = room_map[row.room_id[0]].position;
            msg.door_status = row.door_status; 
            pub.publish(msg);    // publish message     
            ROS_INFO_STREAM("publish a message: " <<msg);                  
        }
        return true;
    }

private:
    MYSQL *connection;
    ros::NodeHandle nh;
    std::map<char,geometry_msgs::Pose> room_map;
    SQLClient sql_client;
    ros::Publisher pub;   
};

int main(int argc, char** argv){
    ROS_INFO("main function start\n");
    ros::init(argc,argv,"door_status_advertiser"); // Initializes Node Name
    Advertiser ad;      
    return 0;
}
