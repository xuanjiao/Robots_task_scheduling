#include "ros/ros.h"
#include <signal.h>
#include <stdio.h>
#include <mysql/mysql.h>
#include <sstream>
#include "util.h"
using namespace std;
class Advertiser{
    public:
    Advertiser(){
        init_time();
        connect_to_db();
        query();

        ros::spin();
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

        ROS_INFO_STREAM("start time: "<<rawtime);
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

    void query(){
            MYSQL_RES *result;  
            MYSQL_ROW row; 

            // std::stringstream query;
            char room_id = 'a';
            char query[200];
            std::string cur_time =Util::time_str(ros::Time::now());
            sprintf(query,"SELECT *  FROM door_status_list WHERE room_id = '%c' and TIMEDIFF(\"%s\",date_time)<\"01:00:00\" and TIMEDIFF(\" %s \",date_time)>=0",room_id,cur_time.c_str(),cur_time.c_str());
        
            ROS_INFO_STREAM(query);

        if(mysql_query(connection, query))  
        {
            ROS_INFO("Query Error: %s", mysql_error(connection));  
            exit(1);  
        }
        else{
            // result = mysql_store_result(connection);
            result = mysql_use_result(connection);
            row = mysql_fetch_row(result); 
   
            if(row <= 0){  
                ROS_INFO("no result");
                return; 
            }
            bool door_status = false;
            door_status = (std::strcmp(row[1],"0")==0)?false:true;  // get first row in result
            ROS_INFO("door %s",door_status?"opened":"closed");
                    
            mysql_free_result(result);  
        }
    }

private:
    MYSQL *connection;
    ros::NodeHandle nh;
};
int main(int argc, char** argv){
    ROS_INFO("main function start\n");
    ros::init(argc,argv,"door_status_advertiser"); // Initializes Node Name
    Advertiser ad;
    
      
    return 0;
}
