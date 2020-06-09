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
  double open_pos;
  double statistuc_open_pos; 
  int time_slot_left;
  int time_slot_right;
  int day_of_week;
}Table_row;

typedef struct list_row{
  std::string room_id;
  std::string date_time;
  bool door_status;
}List_row;



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
    
      update_possibility_table_statement = con->prepareStatement("	update " + std::string(TABLE_NAME) + 
		                                      " set statistic_door_open_posibility = ( \
			                                        select 100*sum(door_status)/count(door_status) \
						                                          from " + std::string(LIST_NAME) +
		      	" where ? = room_id and dayofweek(?) = dayofweek(date_time) and (TIME(date_time) between start_time and end_time))\
	          where  ? = room_id and dayofweek(?) = day_of_week and (time(?) between start_time and end_time);"
     );
    
    
    }

    bool update_possibility_table(list_row &row){
        update_possibility_table_statement->setString(1,row.room_id);
        update_possibility_table_statement->setString(2,row.date_time);
        update_possibility_table_statement->setString(3,row.room_id);
        update_possibility_table_statement->setString(4,row.date_time);
        update_possibility_table_statement->setString(5,row.date_time);
        update_possibility_table_statement->executeUpdate();
    }

    bool insert_to_list(List_row &row){        
      insert_list_statement->setString(1,row.room_id);
      insert_list_statement->setBoolean(2,row.door_status);
      insert_list_statement->setString(3,row.date_time);
      insert_list_statement->execute();
    }



    bool query_posibility_table_single_room(Table_row& table_row,List_row& list_row){
      
      try
      {      
        query_table_single_room_statement->setString(1,list_row.date_time);
        query_table_single_room_statement->setString(2,list_row.date_time);
        query_table_single_room_statement->setString(3,list_row.room_id);
        result_set =  query_table_single_room_statement->executeQuery();
        if(result_set->rowsCount()==0){
            ROS_INFO_STREAM("no result");
            return false;
        }

        while (result_set->next())
        {
            table_row.room_id = result_set->getString("room_id");
            table_row.day_of_week = result_set->getInt("day_of_week");
            table_row.time_slot_left = result_set->getInt("start_time");
            table_row.time_slot_right = result_set->getInt("end_time");
            table_row.open_pos = result_set->getDouble("open_posibility");
            table_row.statistuc_open_pos = result_set->getDouble("statistic_door_open_posibility");
            list_row.door_status = rand()%100<table_row.open_pos?1:0;                    
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
    bool query_posibility_table_rooms(std::vector<Table_row>& table_rows, std::vector<List_row>& list_rows,ros::Time t){
      std::string time = Util::time_str(t);
      try
      {     
         query_table_rooms_statement->setString(1,time);
         query_table_rooms_statement->setString(2,time);
        
        result_set =  query_table_rooms_statement->executeQuery();
        if(result_set->rowsCount()==0){
            ROS_INFO_STREAM("no result");
            return false;
        }

        while (result_set->next())
        {
            Table_row table_row;
            List_row list_row;
            
            table_row.room_id = result_set->getString("room_id");
            table_row.day_of_week = result_set->getInt("day_of_week");
            table_row.time_slot_left = result_set->getInt("start_time");
            table_row.time_slot_right = result_set->getInt("end_time");
            table_row.open_pos = result_set->getDouble("open_posibility");
            table_row.statistuc_open_pos = result_set->getDouble("statistic_door_open_posibility");
            table_rows.push_back(table_row);
            list_row.room_id = result_set->getString("room_id");
            list_row.door_status = rand()%100<table_row.open_pos?1:0;  
            list_rows.push_back(list_row);
            ROS_INFO_STREAM("open possibility "<<table_row.open_pos << " generate door status "<<list_row.door_status);

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

    ~SQLClient(){
      delete insert_list_statement;
      delete query_table_rooms_statement;
      delete query_table_single_room_statement;
      delete update_possibility_table_statement;
      delete result_set;
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
    sql::PreparedStatement* update_possibility_table_statement;  
    sql::ResultSet* result_set;
};
