#pragma once

#include <stdlib.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <mysql/jdbc.h>
#include "util.h"
#include <list>

#define   POSSIBILITY_TABLE                               "open_possibilities"
#define   DOOR_STATUS_LIST                                "door_status_list"
#define   TASK_TABLE_NAME                          "tasks"
#define   CHARGING_STATION_POSITION_TABLE                 "charging_station_position"
#define   ROOM_POSITION_TABLE                             "door_position"
#define   DATABASE_NAME                                   "sensor_db"
#define   URI                                             "tcp://127.0.0.1"

using namespace std;

typedef struct {
  string room_id;
  double open_pos;
  double statistuc_open_pos; 
  int time_slot_left;
  int time_slot_right;
  int day_of_week;
}PossibilityTableRow;

typedef struct {
  string room_id;
  string date_time;
  bool door_status;
}DoorStatusListRow;

typedef struct {
  string room_id;
  string date_time;
  bool door_status;
}PositionTableRow;

class SQLClient{
  public:
    SQLClient(string user_name, string pass):user_name(user_name),pass(pass){
      connect_to_database();
      //prepare_statements();
    }
      
    void connect_to_database(){
      driver = sql::mysql::get_driver_instance();
      con = driver->connect(URI,user_name,pass);
      if(con->isValid()){
        ROS_INFO_STREAM("Connected to "<< DATABASE_NAME);
        con->setSchema(DATABASE_NAME);
      }else{
        ROS_INFO_STREAM("Connected to "<< DATABASE_NAME<<" failed");
        exit(1);
      }
      stmt = con->createStatement();
    }

    void print_table(string table_name){
      
      sql::ResultSet *res;
      stringstream ss;
      list<sql::SQLString> column_names;
      try{         
          ss << "\n-----------------------------------------------------------------------\n";
            res = stmt->executeQuery("SHOW COLUMNS FROM " + table_name);
            while (res->next()){
              column_names.push_back(res->getString("Field"));   
              ss << res->getString("Field")<< setw(15);
            }
            ss << "\n---------------------------------------------------------------------\n";
            res = stmt->executeQuery("SELECT * FROM " + table_name);
            while(res->next()){
              for(sql::SQLString c : column_names){
                  ss << res->getString(c)<<"  ";
              }
              ss<< "\n";
            }
      }catch(const sql::SQLException &e){
        ROS_INFO_STREAM(e.what());
        exit(1);
      }    
      delete res;
      ROS_INFO_STREAM(ss.str());
    }

    int query_multiple_target_position(map<char,geometry_msgs::Pose> &map,std::string target_type){
      sql::ResultSet* res;
      int ret = -1;
      try
      { 
        res = stmt->executeQuery("SELECT * FROM targets WHERE target_type = " + target_type);
        ret = res->rowsCount();
        ROS_INFO_STREAM("Found " << ret << " target type "<< target_type);
        while(res->next()){
          char id = res->getString("target_id")[0];
          geometry_msgs::Pose pose;
          pose.position.x = res->getDouble("position_x");
          pose.position.y = res->getDouble("position_y");
          pose.orientation.z = res->getDouble("orientation_z");
          pose.orientation.w = res->getDouble("orientation_w");
          map.insert(pair<char,geometry_msgs::Pose>(id,pose)) ;
        }
      }
      catch(const exception& e)
      {
        ROS_INFO_STREAM( e.what() );
        delete res; 
        exit(1);
      }      
        delete res; 
        return ret;
    }

    geometry_msgs::Pose query_target_position(string target_id){
       sql::ResultSet* res;
       geometry_msgs::Pose pose;
      try
      { 
        res = stmt->executeQuery("SELECT * FROM targets WHERE target_id = " + target_id + " LIMIT 1");
          res->next();
          pose.position.x = res->getDouble("position_x");
          pose.position.y = res->getDouble("position_y");
          pose.orientation.z = res->getDouble("orientation_z");
          pose.orientation.w = res->getDouble("orientation_w");       
      }
      catch(const sql::SQLException& e)
      {
        ROS_INFO_STREAM( e.what() );
      }
        
        delete res; 
        return pose;
    }

    vector<tuple<char,geometry_msgs::Pose,long double>>
    query_target_pose_and_open_pos_st(string time){
      sql::ResultSet* res;
      vector<tuple<char,geometry_msgs::Pose,long double>> v;
      try{
        res = stmt->executeQuery("select t.target_id,t.position_x, t.position_y, t.orientation_w, t.orientation_z, o.open_pos \
                                      from targets t \
                                      inner join open_possibilities o \
                    where t.target_id = o.door_id  and o.day_of_week = dayofweek('" + time +  "') and time('" + time +  "') between o.start_time and o.end_time; ");
      }catch(sql::SQLException e){
        ROS_INFO_STREAM( e.what() );
      }
      while(res->next()){
        geometry_msgs::Pose pose;
        pose.position.x = res->getDouble("position_x");
        pose.position.y = res->getDouble("position_y");
        pose.orientation.z = res->getDouble("orientation_z");
        pose.orientation.w = res->getDouble("orientation_w");
        v.push_back(
          tuple<char,geometry_msgs::Pose,long double>(
            res->getString("target_id")[0], pose, res->getDouble("open_pos")
          )
        );
      }
      delete res;
      return v;
    }

    void insert_new_enter_room_tasks(int num, string start_time){
      sql::ResultSet* res;
      vector<char> doors;
      char id,priority;
      res = stmt->executeQuery("SELECT target_id  FROM targets WHERE target_type = 'Door'");
      while(res->next()){
        doors.push_back(res->getString("target_id")[0]); // find available door id
      }
      for(int i = 0; i < num; i++){
        priority = rand()%4  + '1';     // 1-5
        id = doors[rand()%doors.size()];  
        stmt->execute(
            "INSERT INTO tasks(task_type, start_time, target_id, priority) VALUES('EnterRoom','" + start_time + "','" + id + "'," + priority +")"
        );
      }
    }

    vector<tuple<char,geometry_msgs::Pose,string,double>>
    query_all_task_pose_time_open_pos(){
      vector<tuple<char,geometry_msgs::Pose,string,double>> v;
      // stmt->executeQuery("SELECT *")
    }

    // void prepare_statements(){
    //   query_table_rooms_statement = con->prepareStatement("select * from " + string(POSSIBILITY_TABLE) +                             
		//      " where (TIME(?) between start_time and end_time )and dayofweek(?) = day_of_week "
    //   );
    //   query_table_single_room_statement = con->prepareStatement("select * from " + string(POSSIBILITY_TABLE) +                             
		//      " where (TIME(?) between start_time and end_time )and dayofweek(?) = day_of_week and ? = room_id"
    //   );
    //   insert_list_statement = con->prepareStatement( "insert into " + string(DOOR_STATUS_LIST)+" values (?,?,?)");
    
    //   update_possibility_table_statement = con->prepareStatement("	update " + string(POSSIBILITY_TABLE) + 
		//                                       " set statistic_door_open_posibility = ( \
		// 	                                        select 100*sum(door_status)/count(door_status) \
		// 				                                          from " + string(DOOR_STATUS_LIST) +
		//       	" where ? = room_id and dayofweek(?) = dayofweek(date_time) and (TIME(date_time) between start_time and end_time))\
	  //         where  ? = room_id and dayofweek(?) = day_of_week and (time(?) between start_time and end_time);"
    //  );

    //  query_rooms_position_statement = con->prepareStatement(" select * from " + string(ROOM_POSITION_TABLE));
    //  query_charging_stations_position_statement = con->prepareStatement(" select * from " + string(CHARGING_STATION_POSITION_TABLE));
    // }

    // int query_charging_stations_position(map<char,geometry_msgs::Pose> &station_map){
    //     sql::ResultSet* res;
    //     int ret = -1;
    //   try{
    //      res = query_charging_stations_position_statement->executeQuery();
    //      ret = res->rowsCount();
    //     while(res->next()){
    //       char id = res->getString("station_id")[0];
    //       geometry_msgs::Pose pose;
    //       pose.position.x = res->getDouble("position_x");
    //       pose.position.y = res->getDouble("position_y");
    //       pose.orientation.z = res->getDouble("orientation_z");
    //       pose.orientation.w = res->getDouble("orientation_w");
    //       station_map.insert(pair<char,geometry_msgs::Pose>(id,pose)) ;
    //     }
    //   }catch (const exception& e){
    //     ROS_INFO_STREAM( e.what());
    //   }
    //     delete res;  
    //     return ret;   
    // }

    // int query_rooms_position(map<char,geometry_msgs::Pose> &room_map){
    //   sql::ResultSet* res;
    //   int ret = -1;
    //   try
    //   { 
    //     res = query_rooms_position_statement->executeQuery();
    //     ret = res->rowsCount();
    //     while(res->next()){
    //       char id = res->getString("room_id")[0];
    //       geometry_msgs::Pose pose;
    //       pose.position.x = res->getDouble("position_x");
    //       pose.position.y = res->getDouble("position_y");
    //       pose.orientation.z = res->getDouble("orientation_z");
    //       pose.orientation.w = res->getDouble("orientation_w");
    //       room_map.insert(pair<char,geometry_msgs::Pose>(id,pose)) ;
    //     }
    //   }
    //   catch(const exception& e)
    //   {
    //     ROS_INFO_STREAM( e.what() );
    //   }
        
    //     delete res; 
    //     return ret;
    // }


    // void update_possibility_table(DoorStatusListRow &row){
    //     update_possibility_table_statement->setString(1,row.room_id);
    //     update_possibility_table_statement->setString(2,row.date_time);
    //     update_possibility_table_statement->setString(3,row.room_id);
    //     update_possibility_table_statement->setString(4,row.date_time);
    //     update_possibility_table_statement->setString(5,row.date_time);
    //     update_possibility_table_statement->executeUpdate();
    // }

    // void insert_to_list(DoorStatusListRow &row){        
    //   insert_list_statement->setString(1,row.room_id);
    //   insert_list_statement->setBoolean(2,row.door_status);
    //   insert_list_statement->setString(3,row.date_time);
    //   insert_list_statement->execute();
    // }

    // int query_posibility_table_single_room(PossibilityTableRow& table_row,DoorStatusListRow& list_row){
    //    sql::ResultSet* res;
    //    int ret = -1;
    //   try
    //   {      
    //     query_table_single_room_statement->setString(1,list_row.date_time);
    //     query_table_single_room_statement->setString(2,list_row.date_time);
    //     query_table_single_room_statement->setString(3,list_row.room_id);
    //     res =  query_table_single_room_statement->executeQuery();
    //     ret = res->rowsCount();
    //     if(ret <=0){
    //         ROS_INFO_STREAM("no result");
    //         return ret;
    //     }

    //     while (res->next())
    //     {
    //         table_row.room_id = res->getString("room_id");
    //         table_row.day_of_week = res->getInt("day_of_week");
    //         table_row.time_slot_left = res->getInt("start_time");
    //         table_row.time_slot_right = res->getInt("end_time");
    //         table_row.open_pos = res->getDouble("open_posibility");
    //         table_row.statistuc_open_pos = res->getDouble("statistic_door_open_posibility");
    //         list_row.door_status = rand()%100<table_row.open_pos?1:0;                    
    //     }
    //   }
    //   catch(sql::SQLException &e)
    //   {
    //     ROS_INFO_STREAM( e.what());
    //   }catch (const exception& e){
    //     ROS_INFO_STREAM( e.what());
    //   }
    //   delete res;
    //   return ret;
      
    // }

    // int query_posibility_table_rooms(vector<PossibilityTableRow>& table_rows, vector<DoorStatusListRow>& list_rows,ros::Time t){
    //       sql::ResultSet* res;
    //       int ret = -1;
    //       string time = Util::time_str(t);
    //   try
    //   {     
    //      query_table_rooms_statement->setString(1,time);
    //      query_table_rooms_statement->setString(2,time);
        
    //     res =  query_table_rooms_statement->executeQuery();
    //     ret = res->rowsCount();
    //     if(ret <=0){
    //         ROS_INFO_STREAM("no result");
    //         return ret;
    //     }

    //     while (res->next())
    //     {
    //         PossibilityTableRow table_row;
    //         DoorStatusListRow list_row;
            
    //         table_row.room_id = res->getString("room_id");
    //         table_row.day_of_week = res->getInt("day_of_week");
    //         table_row.time_slot_left = res->getInt("start_time");
    //         table_row.time_slot_right = res->getInt("end_time");
    //         table_row.open_pos = res->getDouble("open_posibility");
    //         table_row.statistuc_open_pos = res->getDouble("statistic_door_open_posibility");
    //         table_rows.push_back(table_row);
    //         list_row.room_id = res->getString("room_id");
    //         list_row.door_status = rand()%100<table_row.open_pos?1:0;  
    //         list_rows.push_back(list_row);
    //         // ROS_INFO_STREAM("open possibility "<<table_row.open_pos << " generate door status "<<list_row.door_status);

    //     }
    //   }
    //   catch(sql::SQLException &e)
    //   {
    //     ROS_INFO_STREAM( e.what());
    //     return false;
    //   }catch (const exception& e){
    //     ROS_INFO_STREAM( e.what());
    //   }
    //   delete res;
    //   return true;
      
    // }

    ~SQLClient(){
       delete stmt;
      // delete insert_list_statement;
      // delete query_table_rooms_statement;
      // delete query_table_single_room_statement;
      // delete update_possibility_table_statement;
      // delete query_rooms_position_statement;
      // delete query_charging_stations_position_statement;
    }

   private:
    string user_name;
    string pass;
    sql::Driver* driver;
    sql::Connection* con;
    sql::Statement* stmt;
    // sql::PreparedStatement* insert_list_statement;
    // sql::PreparedStatement* query_table_rooms_statement;
    // sql::PreparedStatement* query_table_single_room_statement;
    // sql::PreparedStatement* update_possibility_table_statement;  
    // sql::PreparedStatement* query_rooms_position_statement;
    // sql::PreparedStatement* query_charging_stations_position_statement;

};
