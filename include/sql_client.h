#pragma once

#include <stdlib.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <mysql/jdbc.h>
#include "util.h"

#define   USER_NAME "root"
#define   PASSWARD  "pi"
#define   TABLE_NAME "open_possibility_table"
#define   LIST_NAME "door_status_list"
#define   DATABASE_NAME  "sensor_db"
#define   URI "tcp://127.0.0.1"


typedef struct table_row{
  std::string room_id;
  int open_pos;
  int statistuc_open_pos; 
  int time_slot_left;
  int time_slot_right;
  int day_of_week;
  bool door_status;
}Table_row;



class SQLClient{
  public:
    SQLClient(){
      connect_to_database();
      prepare_statements();
    }

    void connect_to_database(){
      driver = sql::mysql::get_driver_instance();
      con = driver->connect(URI,USER_NAME,PASSWARD);
      if(con->isValid()){
        ROS_INFO_STREAM("Connected to "<< DATABASE_NAME);
        con->setSchema(DATABASE_NAME);
      }else{
        ROS_INFO_STREAM("Connected to "<< DATABASE_NAME<<" failed");
        exit(1);
      }

    }

    void prepare_statements(){
      query_table_rooms_statement = con->prepareStatement("select * from " + std::string(TABLE_NAME) +                             
		     " where (TIME(?) between start_time and end_time )and dayofweek(?) = day_of_week "
      );
      query_table_single_room_statement = con->prepareStatement("select * from " + std::string(TABLE_NAME) +                             
		     " where (TIME(?) between start_time and end_time )and dayofweek(?) = day_of_week and ? = room_id"
      );
      insert_list_statement = con->prepareStatement( "insert into " + std::string(LIST_NAME)+" values (?,?,?)");
    }

    bool query_posibility_table_single_room(Table_row& row, const ros::Time& t){
      std::string time = Util::time_str(t);
      try
      {      
        query_table_single_room_statement->setString(1,time);
        query_table_single_room_statement->setString(2,time);
        query_table_single_room_statement->setString(3,row.room_id);
        result =  query_table_single_room_statement->executeQuery();
        if(result->rowsCount()==0){
            ROS_INFO_STREAM("no result");
            return false;
        }

        while (result->next())
        {
            
            row.room_id = result->getString("room_id");
            row.day_of_week = result->getInt("day_of_week");
            row.time_slot_left = result->getInt("start_time");
            row.time_slot_right = result->getInt("end_time");
            row.open_pos = result->getInt("open_posibility");
            row.statistuc_open_pos = result->getInt("statistic_door_open_posibility");
            row.door_status = rand()%100<row.open_pos?1:0;                    
        }
      }
      catch(sql::SQLException &e)
      {
        ROS_INFO_STREAM( e.what());
        return false;
      }catch (const std::exception& e){
        ROS_INFO_STREAM( e.what());
      }
      return true;
      
    }
    bool query_posibility_table_rooms(std::vector<Table_row>& rows, const ros::Time& t){
      std::string time = Util::time_str(t);
      try
      {     
         query_table_rooms_statement->setString(1,time);
         query_table_rooms_statement->setString(2,time);
        
        result =  query_table_rooms_statement->executeQuery();
        if(result->rowsCount()==0){
            ROS_INFO_STREAM("no result");
            return false;
        }

        while (result->next())
        {
            Table_row row;
            row.room_id = result->getString("room_id");
            row.day_of_week = result->getInt("day_of_week");
            row.time_slot_left = result->getInt("start_time");
            row.time_slot_right = result->getInt("end_time");
            row.open_pos = result->getInt("open_posibility");
            row.statistuc_open_pos = result->getInt("statistic_door_open_posibility");
            rows.push_back(row);
            row.door_status = rand()%100<row.open_pos?1:0;                    
        }
      }
      catch(sql::SQLException &e)
      {
        ROS_INFO_STREAM( e.what());
        return false;
      }catch (const std::exception& e){
        ROS_INFO_STREAM( e.what());
      }
      return true;
      
    }

    bool update_possibility_table(){
        // for(Table_row row :rows){
        //     insert_list_statement->setString(1,LIST_NAME);
        //     insert_list_statement->setString(2,row.room_id);
        //     insert_list_statement->setBoolean(3,row.door_status);
        //     insert_list_statement->setString(4,time);

        //     insert_list_statement->execute();
        // }
    }


    ~SQLClient(){
      delete insert_list_statement;
      delete query_table_rooms_statement;
      delete query_table_single_room_statement;
    }

   static SQLClient& getInstance() {
      static SQLClient instance;
      return instance;
    }
   private:
    sql::Driver* driver;
    sql::Connection* con;
    sql::PreparedStatement* insert_list_statement;
    sql::PreparedStatement* query_table_rooms_statement;
    sql::PreparedStatement* query_table_single_room_statement;
    sql::ResultSet* result;
  
};
