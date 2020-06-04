/* Standard C++ includes */
#include <stdlib.h>
#include <iostream>
#include <ros/ros.h>
#include "util.h"
/*
  Include directly the different
  headers from cppconn/ and mysql_driver.h + mysql_util.h
  (and mysql_connection.h). This will reduce your build time!
*/

#include <mysql/jdbc.h>

#define   USER_NAME "root"
#define   PASSWARD  "pi"
#define   DATABASE_NAME  "sensor_db"
#define   URI "tcp://127.0.0.1"


typedef struct table_row{
  char room_id;
  int open_pos;
  int statistuc_open_pos; 
  int time_slot_left;
  int time_slot_right;
  int day_of_week;
}Table_row;



class SQLClient{
  public:
    SQLClient(){
      connect_to_database();
    }

    void connect_to_database(){
      driver = sql::mysql::get_driver_instance();
      con = driver->connect(URI,USER_NAME,PASSWARD);
      if(con->isValid()){
        ROS_INFO_STREAM("Connected to "<< DATABASE_NAME);
        con->setSchema(DATABASE_NAME);
        stmt = con->createStatement();
      }else{
        ROS_INFO_STREAM("Connected to "<< DATABASE_NAME<<" failed");
        exit(1);
      }
    }

    void create_database_table(){


    }

    void query_posibility_table(Table_row& row, const ros::Time& t){
      std::string time = Util::time_str(t);
      try
      {
        query_table_statement = con->prepareStatement("select *\
                                  from open_possibility_table \
		     where (TIME(?) between start_time and end_time )and dayofweek(?) = day_of_week and room_id = ? ");
        ROS_INFO_STREAM("query parameters: "<<row.room_id<<" "<<time);
        query_table_statement->setString(1,time);
        query_table_statement->setString(2,std::string(1,row.room_id));
        query_table_statement->setString(3,time);
        
        result = query_table_statement->executeQuery();
        if(result->rowsCount()==0){
            ROS_INFO_STREAM("no result");
            return;
        }
        while (result->next())
        {
            row.day_of_week = result->getInt("day_of_week");
            row.time_slot_left = result->getInt("start_time");
            row.time_slot_right = result->getInt("end_time");
            row.open_pos = result->getInt("open_posibility");
            row.statistuc_open_pos = result->getInt("statistic_door_open_posibility");
          ROS_INFO_STREAM("room id =  "         <<  row.room_id<<
                          "day of week = "      <<  row.day_of_week<<
                          "time_slot_left = "   <<  row.time_slot_left<<
                          "time_slot_right = "  <<  row.time_slot_right<<
                          "open possibility = " <<  row.open_pos <<
                          "statistic_door_open_posibility = "  <<  row.statistuc_open_pos);                       
        }
      }
      catch(sql::SQLException &e)
      {
        ROS_INFO_STREAM( e.what());
      }catch (const std::exception& e){
        ROS_INFO_STREAM( e.what());
      }
      
    }
    ~SQLClient(){
      delete query_table_statement;
    }

   static SQLClient& getInstance() {
      static SQLClient instance;
      return instance;
    }
   private:
    sql::Driver* driver;
    sql::Connection *con;
    
    sql::Statement *stmt;
    sql::PreparedStatement *query_table_statement;
    sql::ResultSet* result;

   
};

int main(int argc, char**argv){
  ros::init(argc,argv,"sql_client");
  ros::Time::init();
  ros::Duration(1).sleep();
  ros::NodeHandle nh;
  SQLClient sql_client = SQLClient::getInstance();
  Table_row row;
  row.room_id = 'a';
  sql_client.query_posibility_table(row,ros::Time::now());

  ros::spin();
}