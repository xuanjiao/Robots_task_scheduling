#pragma once

#include <stdlib.h>
#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <sstream>
#include "mysql_connection.h"
#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include "task_type.h"
#include "door.h"
#include "charging_station.h"
#include "util.h"
#include <boost/thread/mutex.hpp>

#define   DATABASE_NAME                                   "sensor_db"
#define   URI                                             "tcp://127.0.0.1"

using namespace std;

class SQLClient{
  public:
  SQLClient(){}
  SQLClient(string user_name, string pass){
    ConnectToDatabase(user_name,pass);
  }

  void ConnectToDatabase(string user_name, string pass){
    _sqlMtx.lock();
    _driver = get_driver_instance();
    _con = _driver->connect(URI,user_name,pass);
    if(_con->isValid()){
      ROS_INFO_STREAM("Connected to "<< DATABASE_NAME);
      _con->setSchema(DATABASE_NAME);
    }else{
      ROS_INFO_STREAM("Connected to "<< DATABASE_NAME<<" failed");
    }
    stmt = _con->createStatement();
    _sqlMtx.unlock();
  }

    // truncate table
    void TruncateTable(string name){
      _sqlMtx.lock();
      stmt->execute("TRUNCATE "+name);
      _sqlMtx.unlock();
    }

    // Print table include columns and data
    void PrintTable(string table_name){
      _sqlMtx.lock();
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
      _sqlMtx.unlock();
    }

    vector<tuple<int,geometry_msgs::Pose,long double>>
    QueryTargetPositionAndOpenPossibilities(string time){
      _sqlMtx.lock();
      sql::ResultSet* res;
      vector<tuple<int,geometry_msgs::Pose,long double>> v;
      try{
        res = stmt->executeQuery("select t.target_id,t.position_x, t.position_y, o.open_pos \
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
          pose.orientation.w = 1.0;
          v.push_back(
            tuple<int,geometry_msgs::Pose,long double>(
              res->getInt("target_id"), pose, res->getDouble("open_pos")
            )
          );
        }
      }

      delete res;
      _sqlMtx.unlock();
      return v;
    }
  
      // get task info to calculate cost
  vector<SmallExecuteTask>
  QueryRunableExecuteTasks(){
    _sqlMtx.lock();
    sql::ResultSet* res;
    vector<SmallExecuteTask> v;
    string now = Util::time_str(ros::Time::now());
    res = stmt->executeQuery(
      "SELECT tasks.dependency, tasks.priority, tasks.target_id, tasks.task_id, tasks.task_type, tasks.start_time, \
      tg.position_x, tg.position_y FROM targets tg \
      INNER JOIN tasks ON tasks.target_id = tg.target_id \
      AND tasks.cur_status IN ('Created','ToReRun') \
      AND tasks.task_type = 'ExecuteTask' \
      AND tasks.start_time > '" + now +"'"
    );
    if(res->rowsCount()!=0){
      while(res->next()){
        SmallExecuteTask t;
        t.priority = res->getInt("priority");
        t.targetId = res->getInt("target_id");
        t.taskId = res->getInt("task_id");
        t.taskType = res->getString("task_type");
        t.dependency = res->getInt("dependency");

        t.goal.header.frame_id = "map";
        t.goal.header.stamp = Util::str_ros_time(res->getString("start_time"));
        t.goal.pose.position.x = res->getDouble("position_x");
        t.goal.pose.position.y = res->getDouble("position_y");
        t.goal.pose.orientation.w = 1.0;
        v.push_back(t);
      } 
    }

    delete res;
    _sqlMtx.unlock();
    return v;
  }

  vector<Door> QueryDoorInfo(){
    _sqlMtx.lock();
    sql::ResultSet* res;
    vector<Door> doors;
    string time = Util::time_str(ros::Time::now());
    res = stmt->executeQuery(
      "SELECT i.door_id, i.dependency, i.last_update, i.is_used, t.position_x, t.position_y, o.open_pos_st FROM door_infos i \
        INNER JOIN targets t ON t.target_id = i.door_id \
        LEFT JOIN (SELECT * FROM open_possibilities \
        WHERE day_of_week = DAYOFWEEK('" + time +  "') and time('" + time +"') between start_time and end_time) o \
        ON i.dependency = o.door_id \
        ORDER BY i.door_id"
    );

    if(res->rowsCount() == 0){
      ROS_INFO_STREAM("No Door Info");
      return doors;
    }
   
    while(res->next()){
      Door d;
      d.doorId = res->getInt("door_id"); // find available door id
      d.lastUpdate = Util::str_ros_time(res->getString("last_update"));
      if(res->getInt("dependency")!=0)
        d.depOpenpossibility = res->getDouble("open_pos_st");
        d.pose.position.x = res->getDouble("position_x");
        d.pose.position.y = res->getDouble("position_y");
        d.pose.orientation.w = 1.0;
        d.isUsed = res->getBoolean("is_used");
        doors.push_back(d);
    }
    delete res;
    
  _sqlMtx.unlock();
    return doors;
  }

  ChargingStation QueryChargingStationInfo(int stationId){
    _sqlMtx.lock();
    sql::ResultSet* res;
    ChargingStation cs;
    res = stmt->executeQuery( 
      "SELECT t.position_x, t.position_y, cs.station_id, cs.robot_battery_level, cs.remaining_time \
      FROM charging_stations cs \
      INNER JOIN targets t ON t.target_id = cs.station_id \
      WHERE cs.station_id = "+to_string(stationId));
    if(res->rowsCount() == 0){
      ROS_INFO_STREAM("No Charging Station Info");
      return cs;
    }
   
    res->next();
    cs.stationId = res->getInt("station_id"); 
    cs.batteryLevel = res->getDouble("robot_battery_level");
    cs.remainingTime = res->getDouble("remaining_time");
    cs.pose.position.x = res->getDouble("position_x");
    cs.pose.position.y = res->getDouble("position_y");
    cs.pose.orientation.w = 1.0;

    delete res;
    _sqlMtx.unlock();
    return cs;
  }

  vector<ChargingStation> QueryChargingStationInfo(){
    _sqlMtx.lock();
    sql::ResultSet* res;
    vector<ChargingStation> css;
    res = stmt->executeQuery( 
      "SELECT t.position_x, t.position_y, cs.station_id, cs.robot_battery_level, TIME_TO_SEC(cs.remaining_time) AS t \
      FROM charging_stations cs \
      INNER JOIN targets t ON t.target_id = cs.station_id"
    );
    if(res->rowsCount() == 0){
      ROS_INFO_STREAM("No Charging Station Info");
      return css;
    }
   
    while(res->next()){
      ChargingStation cs;
      cs.stationId = res->getInt("station_id"); 
      cs.batteryLevel = res->getInt("robot_battery_level");
      cs.remainingTime = res->getInt64("t");
      cs.pose.position.x = res->getDouble("position_x");
      cs.pose.position.y = res->getDouble("position_y");
      cs.pose.orientation.w = 1.0;
      css.push_back(cs);
    }
    delete res;
    
  _sqlMtx.unlock();
    return css;
  }

  
  // Create new task and return its task id
  int InsertATaskAssignId(SmallExecuteTask& t){
    _sqlMtx.lock();
      sql::ResultSet* res;        
      stmt->execute(
        "INSERT INTO tasks(dependency,task_type, priority, target_id, start_time) VALUES('"
        +to_string(t.dependency) +"','" + t.taskType +"','" + to_string(t.priority) +"','" + to_string(t.targetId) + "','" + Util::time_str(t.goal.header.stamp)+"')"
        
        );
      res = stmt->executeQuery("SELECT last_insert_id() as id");
      res->next();
      t.taskId = res->getInt("id");
      delete res;
      _sqlMtx.unlock();
      return t.taskId;
  }

  int InsertATaskAssignId(SmallTask& t){
    _sqlMtx.lock();
    sql::ResultSet* res;        
    stmt->execute(
      "INSERT INTO tasks(task_type, priority, target_id, start_time) VALUES('"
      + t.taskType +"','" + to_string(t.priority) +"','" + to_string(t.targetId) + "','" + Util::time_str(t.goal.header.stamp)+"')"
      
      );
    res = stmt->executeQuery("SELECT last_insert_id() as id");
    res->next();
    t.taskId = res->getInt("id");
    delete res;
    _sqlMtx.unlock();
    return t.taskId;
  }

  int InsertATargetAssignId(geometry_msgs::PoseStamped target, string targetType){
    _sqlMtx.lock();
    int id = 0;
    sql::ResultSet* res;        
    string x = to_string(target.pose.position.x);
    string y = to_string(target.pose.position.y);
    
    ROS_INFO_STREAM("Check target exist ");
    res = stmt->executeQuery(
      "SELECT * FROM targets WHERE position_x = " + x + " AND position_y =" + y
    );
    if(res->rowsCount() == 0){
        ROS_INFO_STREAM("Adding new target (" << x << "," <<y << ")");
        stmt->execute(
          "INSERT INTO targets(target_type, position_x, position_y) \
            VALUES('"+ targetType + "'," + x + "," + y + ")" 
        );
        res = stmt->executeQuery("SELECT last_insert_id() as target_id");
        res->next();
        if(res->rowsCount() == 0){
          ROS_INFO_STREAM("Failed to insert target");
          id = 0;
        }else{
          id = res->getInt("target_id");
        }
    }else{
      ROS_INFO_STREAM("This target is already in targets table");
      res->next();
      id = res->getInt("target_id");
    }
  _sqlMtx.unlock();
      return id;
  }


  // Change time and Priority of a returned task
  int UpdateFailedExecuteTask(const vector<int>& taskIds){
    _sqlMtx.lock();
    stringstream ss;
    ss<<"(";
    for(size_t i = 0 ; i< taskIds.size()-1; i++){
      ss << taskIds[i]<<", ";
    }
    ss<<taskIds[taskIds.size()-1]<<")";

    int ret = stmt->executeUpdate("UPDATE tasks \
      SET priority 	= CASE priority  WHEN 5 THEN 5 ELSE priority + 1 END, \
        start_time 	= CASE priority  WHEN 5 THEN start_time ELSE TIMESTAMPADD(SECOND,60,start_time) END, \
        cur_status 	= CASE priority  WHEN 5 THEN 'Canceled' ELSE 'ToReRun' END, \
        description = CASE priority  WHEN 5 THEN 'Drop task' ELSE 'Task failed. Run again' END \
      WHERE task_id IN " +ss.str());
    _sqlMtx.unlock();
    return ret;
  }

  // Insert a record in door status list
  int InsertDoorStatusRecord(int door_id, ros::Time measure_time,bool door_status){
    _sqlMtx.lock();
    string mst = Util::time_str(measure_time);
    // ROS_INFO_STREAM(" insert "<<to_string(door_id)<<" "<<measure_time<<" "<<door_status);
    int ret = stmt->execute("REPLACE INTO measurements(door_id,door_status,date_time) VALUES('" + to_string(door_id) + "', " +to_string(door_status)+", '"+ mst+"')");
    ret = stmt->getUpdateCount();
    _sqlMtx.unlock();
    return ret;
  }

  // Find all records from this time and day of weeks, and calculate new open possibilities
  int UpdateOpenPossibilities(int door_id, ros::Time measure_time){
    _sqlMtx.lock();

    sql::ResultSet* res;
    string dw,st,et,mst = Util::time_str(measure_time);
    
    // select start time, end time, day of week from open possibility table
    res = stmt->executeQuery(
      "SELECT start_time, end_time, day_of_week FROM open_possibilities WHERE door_id = '" + to_string( door_id)+
      "' AND DAYOFWEEK('" + mst + "') = day_of_week AND TIME('" + mst + "') BETWEEN start_time AND end_time"
    );

    if(res->rowsCount()==0){
      ROS_INFO("Unknow day time");
      return -1;
    }

    res->next();
    st = res->getString("start_time");
    et = res->getString("end_time");
    dw = res->getString("day_of_week");

    ROS_INFO_STREAM("Start time "<<st<<" End time "<<et<<" Day of week "<<dw);

    // update open possibility table
    int ret =  stmt->executeUpdate(
      "UPDATE open_possibilities o \
        SET  o.open_pos_st = (SELECT SUM(door_status) / COUNT(door_status)  FROM  measurements ds \
            WHERE ds.door_id = '" + to_string(door_id) + "' AND DAYOFWEEK(ds.date_time) = '" + dw + 
            "' AND TIME(ds.date_time) BETWEEN '" + st + "' AND '"+ et +
        "') WHERE o.door_id = '" + to_string(door_id) + "' AND o.day_of_week = '" + dw + "' AND o.start_time =' " + st + "' AND o.end_time = '"+ et +"'"       
    );
    _sqlMtx.unlock();

    delete res;
    return ret;
  }

    // Change task status in tasks table
  int UpdateTaskStatus(int taskId,string status){
    _sqlMtx.lock();
    int ret =  stmt->executeUpdate("UPDATE tasks set cur_status = '"+ status + "' WHERE task_id = " + to_string(taskId));
    _sqlMtx.unlock();
    return ret;    
  }
  
  int UpdateTaskDescription(int taskId, string description){
    _sqlMtx.lock();
    int ret =  stmt->executeUpdate("UPDATE tasks set description = '"+ description + "' WHERE task_id = " + to_string(taskId));
    _sqlMtx.unlock();
    return ret;
  }

  int UpdateTaskRobotId(int taskId, int robotId){
    _sqlMtx.lock();
    int ret =  stmt->executeUpdate("UPDATE tasks set robot_id = '"+to_string(robotId) + "' WHERE task_id = " + to_string(taskId));
    _sqlMtx.unlock();
    return ret;
  }

  int UpdateChargingStationInfo(const ChargingStation& cs){
    _sqlMtx.lock();
    int ret = stmt->executeUpdate(
      "UPDATE charging_stations \
      SET robot_battery_level = '" + to_string(cs.batteryLevel) + 
      "' WHERE station_id = "+ to_string(cs.stationId)
    );
    _sqlMtx.unlock();
    return ret;
  }

    ~SQLClient(){
       delete stmt;
    }
    
   private:
    sql::Driver* _driver;
    sql::Connection* _con;
    sql::Statement* stmt;
    boost::mutex _sqlMtx;
};
