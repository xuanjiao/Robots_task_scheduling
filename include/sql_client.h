#pragma once

#include <stdlib.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <mysql/jdbc.h>
#include "util.h"

#define   DATABASE_NAME                                   "sensor_db"
#define   URI                                             "tcp://127.0.0.1"

using namespace std;

class SQLClient{
  public:
    SQLClient(string user_name, string pass):user_name(user_name),pass(pass){
      connect_to_database();
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

    // truncate costs table and task table
    void truncate_costs_tasks(){
      stmt->execute("TRUNCATE tasks");
      stmt->execute("TRUNCATE costs");
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
              ss << res->getString("Field")<< " ";
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
    
    // get task info to calculate cost
    vector<tuple<int,ros::Time,geometry_msgs::Pose>>
    query_all_pose_time_in_costs(){
      sql::ResultSet* res;
    vector<tuple<int,ros::Time,geometry_msgs::Pose>> infos;
      res = stmt->executeQuery(
        " SELECT c.task_id, tasks.start_time, t.position_x, t.position_y, t.orientation_w, t.orientation_z FROM targets t \
        inner join costs c, tasks \
        where tasks.target_id = t.target_id and c.task_id = tasks.task_id"
      );
      while(res->next()){
        geometry_msgs::Pose pose;
        string time = res->getString("start_time");
        pose.position.x = res->getDouble("position_x");
        pose.position.y = res->getDouble("position_y");
        pose.orientation.z = res->getDouble("orientation_z");
        pose.orientation.w = res->getDouble("orientation_w");
        
        infos.push_back(make_tuple(res->getInt("task_id"),Util::str_ros_time(time),pose));
      }
      delete res;
      return infos;
    }

    // get best task id
    int query_task_id_highest_cost(){
        // Calculate cost
        int task_id = -1;
        sql::ResultSet* res;
        stmt->executeUpdate("UPDATE costs SET cost = 1.0 * distance + 0.2 * time_diff + (-1.0) * open_pos_st +(-10) * priority  + (-1.0) * battery");
        res = stmt->executeQuery("SELECT task_id FROM costs WHERE cost = (SELECT min(cost) FROM costs)");
        res->next();
        task_id = res->getInt("task_id"); // get task id which has highest cost
        stmt->executeUpdate("UPDATE tasks SET cur_status = 'Running' WHERE task_id = "+to_string(task_id));
        delete res;
        return task_id;
    }
    
    // Create new enter room tasks
    void insert_new_enter_room_tasks(int num, ros::Time start, ros::Duration interval){
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
            "INSERT INTO tasks(task_type, start_time, target_id, priority) VALUES('EnterRoom','" + Util::time_str(start + interval *i) + "','" + id + "'," + priority +")"
        );
      }
      delete res;
    }
    
    // Create new charging task and return its task id
    int insert_new_charging_task(char target_id,ros::Time start){
        sql::ResultSet* res;        
        int id = -1;
        stmt->execute(
          "INSERT INTO tasks(task_type, target_id, start_time) VALUES('Charging','" + string(1,target_id) + "','" + Util::time_str(start)+"')"
          );
        res = stmt->executeQuery("SELECT last_insert_id() as id");
        res->next();
        id = res->getInt("id");
        delete res;
        return id;
    }

    // Create charging task
    vector<pair<char,geometry_msgs::Pose>>
    query_charging_station(){
      vector<pair<char,geometry_msgs::Pose>> v;
      sql::ResultSet* res;
      res = stmt->executeQuery("SELECT * FROM targets WHERE target_type = 'ChargingStation'");
      while(res->next()){
        geometry_msgs::Pose pose;
        pose.position.x = res->getDouble("position_x");
        pose.position.y = res->getDouble("position_y");
        pose.orientation.z = res->getDouble("orientation_z");
        pose.orientation.w = res->getDouble("orientation_w");
        v.push_back(make_pair(res->getString("target_id")[0],pose));
      }
      delete res;
      return v;
    } 

    // insert infomation except distance to cost table
    void insert_available_task_to_costs(ros::Time cur_time, double battery){
        stmt->execute("TRUNCATE costs");
        stmt->execute(
          " INSERT INTO costs(task_id,robot_id,priority, open_pos_st,time_diff, battery) \
            SELECT t.task_id, t.robot_id, t.priority, o.open_pos_st, TIMESTAMPDIFF(SECOND,'" + Util::time_str(cur_time) + "',t.start_time)," + to_string(battery) +" FROM open_possibilities o \
            INNER JOIN tasks t \
            WHERE t.cur_status IN('Created','WaitingToRun') AND t.target_id = o.door_id AND dayofweek(t.start_time) = o.day_of_week AND TIME(t.start_time) BETWEEN o.start_time AND o.end_time"
        );
    }

    // insert distance to cost table
    void update_distances_in_costs(int id,double dist){
          stmt->executeUpdate(
            "UPDATE costs SET distance = " + to_string(dist) + " WHERE task_id = " + to_string(id)
          );
    }

    // returned task
    void update_returned_task(int task_id, ros::Duration d, int pri_inc){
      stmt->executeUpdate("UPDATE tasks set priority = if((priority + " + to_string(pri_inc) +  ")>5,5,priority + "
                + to_string(pri_inc) + ") , cur_status = 'WaitingToRun', start_time  = DATE_ADD(start_time, INTERVAL "+to_string(d.sec)+" SECOND) where task_id = "+to_string(task_id));

    }

    void insert_door_status_list(int task_id){
     // stmt->execute("INSERT INTO")
    }

    void update_task_list_completed(int task_id){
      stmt->executeUpdate("UPDATE tasks set cur_status = 'RanToCompletion' WHERE task_id = " + to_string(task_id));
    }

    void update_expired_tasks_canceled(ros::Time now){
      stmt->executeUpdate("UPDATE tasks set cur_status = 'Canceled' WHERE start_time < '" + Util::time_str(now)+"'");
    }

    ~SQLClient(){
       delete stmt;
    }

   private:
    string user_name;
    string pass;
    sql::Driver* driver;
    sql::Connection* con;
    sql::Statement* stmt;
};
