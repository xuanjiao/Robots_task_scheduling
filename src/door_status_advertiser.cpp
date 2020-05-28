#include "ros/ros.h"
#include <signal.h>
#include <stdio.h>
#include <mysql/mysql.h>
#include <sstream>
#include "util.h"
#include "robot_navigation/sensor_data.h"

using namespace std;
class Advertiser{
    public:
    Advertiser(){
        init_time();
        connect_to_db();
        load_room_positions();

        // publish sensor data
        int max_room;
        nh.getParam("available_room_num",max_room);
        ros::Publisher pub = nh.advertise<robot_navigation::sensor_data>("sensor_data",100);

        //ros::Rate loop_rate(2);
        while(ros::ok()){
            for(int i = 0; i < max_room;i++){
                char room_id = 'a'+i;
                robot_navigation::sensor_data msg;
                msg.stamp = ros::Time::now();
                msg.id = room_id; //convert char to string
                msg.pose = room_map[room_id];
                
                ROS_INFO_STREAM("Current time: "<<Util::time_str(msg.stamp));
                
                if(query(room_id,msg.stamp,msg.door_status)){ //query door value or current time
                    // fond a door status in database
                    // ROS_INFO_STREAM("publish a message: " <<msg);
                    pub.publish(msg);
                }else{
                    ROS_INFO_STREAM("no data for room "<<room_id);
                }
            }
            ROS_INFO_STREAM("sleep");
            ros::spinOnce();
            // loop_rate.sleep();
            ros::Duration(10).sleep();
        }

        

    }

    void init_time(){
        time_t rawtime;
        time(&rawtime);
        struct tm* st = localtime(&rawtime);
        st->tm_year = 2020-1900;
        st->tm_mon = 6-1;
        st->tm_mday = 1;
        st->tm_hour = 8;
        st->tm_min = 0;
        st->tm_sec = 0;
        
        rawtime = mktime(st);

        ros::Time start(rawtime);
        ros::Time::setNow(start);
        ROS_INFO_STREAM("start time: "<<Util::time_str(ros::Time::now()));
    }

    bool connect_to_db(){
        connection = mysql_init(NULL);

        if(connection == NULL){
            ROS_INFO("init error %s\n",mysql_error(connection));
            exit(1);
            return false; 
        }

        connection = mysql_real_connect(connection, "localhost", "root", "pi", "sensor_db", 0, NULL, 0);

        if(connection == NULL){
            ROS_INFO("failed to connect to database error %s",mysql_error(connection));
            exit(1);
            return false; 
        }
    }

    bool query(char room_id,ros::Time time, uint8_t& door_status){
            MYSQL_RES *result;  
            MYSQL_ROW row; 

            char query[200];
            std::string cur_time =Util::time_str(time);
            sprintf(query,"SELECT *  FROM door_status_list WHERE room_id = '%c' and TIMEDIFF(\"%s\",date_time)<\"00:10:00\" and TIMEDIFF(\" %s \",date_time)>=0",room_id,cur_time.c_str(),cur_time.c_str());
        
            ROS_DEBUG_STREAM(query);

        if(mysql_query(connection, query))  
        {
            ROS_INFO("Query Error: %s", mysql_error(connection));  
            exit(1);  
        }else{
            // result = mysql_store_result(connection);
            result = mysql_use_result(connection);
            row = mysql_fetch_row(result); 
   
            if(row <= 0){  
                door_status = false;   // doesn't find log, search result -1
                return false; 
            }
          
            door_status = (std::strcmp(row[1],"0")==0)?0:1;  // get first row in result                            
            mysql_free_result(result);  
            return true;
        }
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
            stream << "/room_pose_" << room_id << "/position";

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
    
};
int main(int argc, char** argv){
    ROS_INFO("main function start\n");
    ros::init(argc,argv,"door_status_advertiser"); // Initializes Node Name
    Advertiser ad;
      
    return 0;
}
