#include "ros/ros.h"
#include <signal.h>
#include <stdio.h>
#include <mysql/mysql.h>
#include <sstream>
#include "util.h"
#include "robot_navigation/sensor_data.h"
#include "sql_client.h"
// #include "time_transfer.h"
// #include "task_process.h"
using namespace std;
class Advertiser{
    public:
    Advertiser():sql_client("door_simulator","pass"){
        ROS_INFO_STREAM("using simulation time "<<ros::Time::isSimTime());
        ROS_INFO_STREAM("using system time "<<ros::Time::isSystemTime());
        ros::Duration(1).sleep();
        ROS_INFO_STREAM("Current time "<<ros::Time::now());
       
        sql_client.connect_to_database();
        sql_client.print_table("targets");
        // load_room_position();
        pub = nh.advertise<robot_navigation::sensor_data>("sensor_data",100);
    }

    void query_publush_door_status(){
        ros::Time now = ros::Time::now();
        string time = Util::time_str(now);
        auto v = sql_client.query_target_pose_and_open_pos_st(time);
        if(!v.size()){
 			ROS_INFO_STREAM("No result. Current time: "<<time);
            exit(1);
        }else{
            for(auto tl : v){   // publish a message for each door
                robot_navigation::sensor_data msg;
                msg.id = get<0>(tl);
                msg.pose = get<1>(tl).position;
                msg.door_status = rand()%100 < (get<2>(tl))?1:0;
                msg.stamp  = now;
                pub.publish(msg);
                ROS_INFO_STREAM(time << "publish a message:\n " <<msg);
            }
        }
    }

private:
    ros::NodeHandle nh;
    SQLClient sql_client;
    ros::Publisher pub;   
};

int main(int argc, char** argv){
    ROS_INFO("main function start\n");
    ros::init(argc,argv,"door_status_advertiser"); // Initializes Node Name
    Advertiser ad;   
    while(ros::ok()){
        ad.query_publush_door_status();
        ros::spinOnce();
        ros::Duration(10).sleep();
    }   
    return 0;
}
