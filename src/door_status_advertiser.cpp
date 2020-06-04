#include "ros/ros.h"
#include <signal.h>
#include <stdio.h>
#include <mysql/mysql.h>
#include <sstream>
#include "util.h"
#include "robot_navigation/sensor_data.h"
#include "sql_client.h"

using namespace std;
class Advertiser{
    public:
    Advertiser():sql_client(SQLClient::getInstance()){
        ROS_INFO_STREAM("using simulation time "<<ros::Time::isSimTime());
        ROS_INFO_STREAM("using system time "<<ros::Time::isSystemTime());
        ros::Duration(1).sleep();
        ROS_INFO_STREAM("Current time "<<Util::time_str(ros::Time::now()));
        load_room_positions();
        sql_client.connect_to_database();
        

        int max_room;
        nh.getParam("available_room_num",max_room);
        pub = nh.advertise<robot_navigation::sensor_data>("sensor_data",100);

        while(ros::ok()){
            publish_door_status();
            ros::spinOnce();
            ros::Duration(10).sleep();
        }
    }

    bool publish_door_status(){
        std::vector<Table_row> rows;
        ros::Time now = ros::Time::now();
        if(!sql_client.query_posibility_table_rooms(rows,now)){
 			ROS_INFO_STREAM("No result. Current time: "<<Util::time_str(now));
            return false;
        }
        for(Table_row row : rows){  
                robot_navigation::sensor_data msg;
                msg.stamp = now;
                msg.id = row.room_id; 
                msg.pose = room_map[row.room_id[0]];
                msg.door_status = row.door_status; 
                pub.publish(msg);    // publish message       
                ROS_INFO_STREAM("publish a message: " <<msg);                  
        }
        return true;
    }

    void load_room_positions(){
        // get position of all rooms store them in map
        std::vector<double> point_vec;
        geometry_msgs::Point point;
        std::stringstream stream;
        char room_id = 'a';
        while(true){
            stream.str("");
            point_vec.clear();
            stream << "/room_" << room_id << "_door/position";

            if(!nh.getParam(stream.str(),point_vec)){ // When room not exist quit
                break;
            }
            point.x = point_vec[0];
            point.y = point_vec[1];
            point.z = point_vec[2];
           
            room_map.insert(std::pair<char,geometry_msgs::Point>(room_id,point));
            room_id++;
        }
        ROS_INFO_STREAM("load "<<room_id-'a'<<" points");
        
    }

private:
    MYSQL *connection;
    ros::NodeHandle nh;
    std::map<char,geometry_msgs::Point> room_map;
    SQLClient sql_client;
    ros::Publisher pub;
    
};
int main(int argc, char** argv){
    ROS_INFO("main function start\n");
    ros::init(argc,argv,"door_status_advertiser"); // Initializes Node Name
    Advertiser ad;
      
    return 0;
}
