#pragma once

#include <stdlib.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <iostream>
#include "mysql_connection.h"
#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>

#include "util.h"

#define   DATABASE_NAME                                   "sensor_db"
#define   URI                                             "tcp://127.0.0.1"

using namespace std;

typedef struct {
    int task_id = 0;
    string task_type = "";
    int target_id = 0;
    double open_pos = 0.0;
    int priority = 0;
    geometry_msgs::PoseStamped goal; // distination and timestamp
    double cost = 0.0;
}Task;


class SQLClient{
  public:
    SQLClient(string user_name, string pass):user_name(user_name),pass(pass){
      ConnectToDatabase();
    }
      
    void ConnectToDatabase(){
      driver = get_driver_instance();
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

    // truncate table
    void TruncateTable(string name){
      stmt->execute("TRUNCATE "+name);
    }

    // Print table include columns and data
    void PrintTable(string table_name){
      
      sql::ResultSet *res;
      stringstream ss;
      list<sql::SQLString> column_names;
      try{         
          ss << "\nPrint table " << table_name << ":\n-----------------------------------------------------------------------\n";
            res = stmt->executeQuery("SHOW COLUMNS FROM " + table_name);
            while (res->next()){
              column_names.push_back(res->getString("Field"));   
              ss << res->getString("Field")<< " ";
            }
            ss << "\n---------------------------------------------------------------------\n";
            res = stmt->executeQuery("SELECT * FROM " + table_name);
            if(res->rowsCount()!=0){
              while(res->next()){
                for(sql::SQLString c : column_names){
                    ss << res->getString(c)<<"  ";
                }
                ss<< "\n";
              }
            }
      }catch(const sql::SQLException &e){
        ROS_INFO_STREAM(e.what());
        exit(1);
      }    
      delete res;
      ROS_INFO_STREAM(ss.str());
    }

    vector<tuple<int,geometry_msgs::Pose,long double>>
    QueryTargetPositionAndOpenPossibilities(string time){
      sql::ResultSet* res;
      vector<tuple<int,geometry_msgs::Pose,long double>> v;
      try{
        res = stmt->executeQuery("select t.target_id,t.position_x, t.position_y, t.orientation_w, t.orientation_z, o.open_pos \
                                      from targets t \
                                      inner join open_possibilities o \
                    where t.target_id = o.door_id  and o.day_of_week = dayofweek('" + time +  "') and time('" + time +  "') between o.start_time and o.end_time; ");
      }catch(sql::SQLException e){
        ROS_INFO_STREAM( e.what() );
      }
      if(res->rowsCount()){
        while(res->next()){
          geometry_msgs::Pose pose;
          pose.position.x = res->getDouble("position_x");
          pose.position.y = res->getDouble("position_y");
          pose.orientation.z = res->getDouble("orientation_z");
          pose.orientation.w = res->getDouble("orientation_w");
          v.push_back(
            tuple<int,geometry_msgs::Pose,long double>(
              res->getInt("target_id"), pose, res->getDouble("open_pos")
            )
          );
        }
      }

      delete res;
      return v;
    }
    
        // get task info to calculate cost
    vector<Task>
    QueryRunableExecuteTasks(){
      sql::ResultSet* res;
      vector<Task> v;
      res = stmt->executeQuery(
       "SELECT tasks.priority, tasks.target_id, tasks.task_id, tasks.task_type, tasks.start_time, \
        tg.position_x, tg.position_y, tg.orientation_z, tg.orientation_w FROM targets tg \
        INNER JOIN tasks ON tasks.target_id = tg.target_id \
        AND tasks.cur_status IN ('Created' ) \
        AND tasks.task_type = 'ExecuteTask'"
      );
      if(res->rowsCount()!=0){
        while(res->next()){
          Task t;
          t.priority = res->getInt("priority");
          t.target_id = res->getInt("target_id");
          t.task_id = res->getInt("task_id");
          t.task_type = res->getString("task_type");

          t.goal.header.frame_id = "map";
          t.goal.header.stamp = Util::str_ros_time(res->getString("start_time"));
          t.goal.pose.position.x = res->getDouble("position_x");
          t.goal.pose.position.y = res->getDouble("position_y");
          t.goal.pose.orientation.z = res->getDouble("orientation_z");
          t.goal.pose.orientation.w = res->getDouble("orientation_w");
          v.push_back(t);
        } 
      }

      delete res;
      return v;
    }

    // get task info to calculate cost
    vector<Task>
    QueryRunableGatherEnviromentInfoTasks(){
      sql::ResultSet* res;
      vector<Task> v;
      res = stmt->executeQuery(
       "SELECT tasks.priority, o.open_pos_st, tasks.target_id, tasks.task_id, tasks.task_type, tasks.start_time, \
        tg.position_x, tg.position_y, tg.orientation_z, tg.orientation_w FROM targets tg \
        INNER JOIN tasks ON tasks.target_id = tg.target_id \
        INNER JOIN open_possibilities o WHERE tg.target_id = o.door_id \
        AND DAYOFWEEK(tasks.start_time) = o.day_of_week \
        AND TIME(tasks.start_time) BETWEEN o.start_time AND o.end_time \
        AND tasks.cur_status IN ('Created' ,'ToReRun') \
        AND tasks.task_type = 'GatherEnviromentInfo'"
      );
      if(res->rowsCount()!=0){
        while(res->next()){
          Task t;
          t.priority = res->getInt("priority");
          t.open_pos = res->getDouble("open_pos_st");
          t.target_id = res->getInt("target_id");
          t.task_id = res->getInt("task_id");
          t.task_type = res->getString("task_type");

          t.goal.header.frame_id = "map";
          t.goal.header.stamp = Util::str_ros_time(res->getString("start_time"));
          t.goal.pose.position.x = res->getDouble("position_x");
          t.goal.pose.position.y = res->getDouble("position_y");
          t.goal.pose.orientation.z = res->getDouble("orientation_z");
          t.goal.pose.orientation.w = res->getDouble("orientation_w");
          v.push_back(t);
        } 
      }

      delete res;
      return v;
    }


    // Create new enter room tasks
    bool InsertMultipleGatherInfoTasks(int num, ros::Time start, ros::Duration interval){
      sql::ResultSet* res;
      bool ret;
      vector<int> doors;
      int id,priority;
      res = stmt->executeQuery("SELECT target_id  FROM targets WHERE target_type = 'Door'");
      while(res->next()){
        doors.push_back(res->getInt("target_id")); // find available door id
      }
      for(int i = 0; i < num; i++){
        priority = rand()%4  + 1;     // 1-5
        id = doors[rand()%doors.size()];  
        ret = stmt->execute(
            "INSERT INTO tasks(task_type, start_time, target_id, priority) VALUES('GatherEnviromentInfo','" + Util::time_str(start + interval *i) + "','" + to_string(id) + "'," + to_string(priority) +")"
        );
      }
      delete res;
      return ret;
    }
    
    // Create new task and return its task id
    int InsertATaskAssignId(Task& t){
        sql::ResultSet* res;        
        stmt->execute(
          "INSERT  IGNORE INTO tasks(task_type, target_id, start_time) VALUES('"+ t.task_type +"','" + to_string(t.target_id) + "','" + Util::time_str(t.goal.header.stamp)+"')"
          );
        res = stmt->executeQuery("SELECT last_insert_id() as id");
        res->next();
        t.task_id = res->getInt("id");
        delete res;
        return t.task_id;
    }

    int InsertATargetAssignId(geometry_msgs::PoseStamped target){
        sql::ResultSet* res;        
        int target_id = -1;
        stmt->execute(
          "INSERT INTO targets(target_type, position_x, position_y, orientation_z, orientation_w) \
            VALUES('Point'," + to_string(target.pose.position.x) + "," + to_string(target.pose.position.y) + "," + to_string(target.pose.orientation.z) +","+to_string(target.pose.orientation.w)
            +")"
        );

        res = stmt->executeQuery("SELECT last_insert_id() as target_id");
        res->next();
        target_id = res->getInt("target_id");

        return target_id;
    }

    // Create charging task
    // vector<pair<int,geometry_msgs::Pose>>
    map<int,geometry_msgs::Pose>
    QueryAvailableChargingStations(){
      // vector<pair<int,geometry_msgs::Pose>> v;
      map<int,geometry_msgs::Pose> map;
      sql::ResultSet* res;
      res = stmt->executeQuery("SELECT * FROM targets tg INNER JOIN charging_stations cs ON tg.target_id = cs.station_id WHERE target_type = 'ChargingStation' AND is_free = 1");
      while(res->next()){
        geometry_msgs::Pose pose;
        pose.position.x = res->getDouble("position_x");
        pose.position.y = res->getDouble("position_y");
        pose.orientation.z = res->getDouble("orientation_z");
        pose.orientation.w = res->getDouble("orientation_w");
        map.insert(make_pair(res->getInt("target_id"),pose));
      }
      delete res;
      return map;
    } 

    // Change time and Priority of a returned task
    void UpdatePriority(int task_id, int pri_inc){
      stmt->executeUpdate("UPDATE tasks set priority = if((priority + " + to_string(pri_inc) +  ")>5,5,priority + "
                + to_string(pri_inc) + ") ");

    }

    // Insert a record in door status list
    bool InsertDoorStatusRecord(int door_id, ros::Time measure_time,bool door_status){
      string mst = Util::time_str(measure_time);
       bool result;
      try{
        // insert new record into door status table
        ROS_INFO_STREAM(" insert "<<to_string(door_id)<<" "<<measure_time<<" "<<door_status);
        result = stmt->execute("INSERT INTO door_status(door_id,door_status,date_time) VALUES('" + to_string(door_id) + "', " +to_string(door_status)+", '"+ mst+"')");
        // PrintTable("door_status");
      }catch(sql::SQLException e){
        ROS_INFO_STREAM(e.what());
      }
      return result;
    }

    // Find all records from this time and day of weeks, and calculate new open possibilities
    void UpdateOpenPossibilities(int door_id, ros::Time measure_time){
      auto t = QueryStartTimeEndTimeDayFromOpenPossibilitiesTable(door_id,measure_time);

      // update open possibility table
      stmt->executeUpdate(
        "UPDATE open_possibilities o \
          SET  o.open_pos_st = (SELECT SUM(door_status) / COUNT(door_status)  FROM  door_status ds \
              WHERE ds.door_id = '" + to_string(door_id) + "' AND DAYOFWEEK(ds.date_time) = '" + to_string(get<2>(t)) + 
              "' AND TIME(ds.date_time) BETWEEN '" + get<0>(t) + "' AND '"+ get<1>(t) +
          "') WHERE o.door_id = '" + to_string(door_id) + "' AND o.day_of_week = '" + to_string(get<2>(t)) + "' AND o.start_time =' " + get<0>(t) + "' AND o.end_time = '"+ get<1>(t) +"'"       
      );
    }

    // Change task status in tasks table
    void UpdateTaskStatus(int task_id,string status){
      stmt->executeUpdate("UPDATE tasks set cur_status = '"+ status + "' WHERE task_id = " + to_string(task_id));
    }

    // Change cancled task status to 'Canceled'
    void UpdateExpiredTask(ros::Time now){
      try{
        stmt->executeUpdate("UPDATE tasks set cur_status = 'Canceled' WHERE start_time < '" + Util::time_str(now)+"' AND task_type = 'GatherEnviromentInfo' AND cur_status IN ('Created','WaitingToRun')");
        stmt->executeUpdate("UPDATE tasks set start_time = '"+Util::time_str(now + ros::Duration(20))+"' WHERE start_time < '" + Util::time_str(now)+"' AND task_type = 'ExecuteTask' AND cur_status NOT IN ('RanToCompletion')");
      }catch(sql::SQLException e){
        ROS_INFO_STREAM(e.what());
        exit(1);
      }
    }

    tuple<string,string,int> QueryStartTimeEndTimeDayFromOpenPossibilitiesTable(int door_id, ros::Time measure_time){
      sql::ResultSet* res;
      string st,et,mst = Util::time_str(measure_time);
      int dw;

      // select start time, end time, day of week from open possibility table
      res = stmt->executeQuery(
        "SELECT start_time, end_time, day_of_week FROM open_possibilities WHERE door_id = '" + to_string( door_id)+
        "' AND DAYOFWEEK('" + mst + "') = day_of_week AND TIME('" + mst + "') BETWEEN start_time AND end_time"
      );

      res->next();
      st = res->getString("start_time");
      et = res->getString("end_time");
      dw = res->getInt("day_of_week");
      delete res;
      return make_tuple(st,et,dw);
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
