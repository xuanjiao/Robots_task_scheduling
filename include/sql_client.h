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
    int task_id;
    string task_type;
    int target_id;
    double open_pos;
    int priority;
    geometry_msgs::PoseStamped goal; // distination and timestamp
    double cost;
}Task;


class SQLClient{
  public:
    SQLClient(string user_name, string pass):user_name(user_name),pass(pass){
      connect_to_database();
    }
      
    void connect_to_database(){
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

    int query_multiple_target_position(map<int,geometry_msgs::Pose> &map,std::string target_type){
      sql::ResultSet* res;
      int ret = -1;
      try
      { 
        res = stmt->executeQuery("SELECT * FROM targets WHERE target_type = " + target_type);
        ret = res->rowsCount();
        ROS_INFO_STREAM("Found " << ret << " target type "<< target_type);
        while(res->next()){
          int id = res->getInt("target_id");
          geometry_msgs::Pose pose;
          pose.position.x = res->getDouble("position_x");
          pose.position.y = res->getDouble("position_y");
          pose.orientation.z = res->getDouble("orientation_z");
          pose.orientation.w = res->getDouble("orientation_w");
          map.insert(pair<int,geometry_msgs::Pose>(id,pose)) ;
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



    // Task query_go_to_point_tasks(){
    //   sql::ResultSet* res;
    //   vector<Task> v;
    //   geometry_msgs::PoseStamped goal;
      
    //   res = stmt->executeQuery(
    //     "SELECT * FROM tasks INNER JOIN targets ON tasks.target_id = targets.target_id WHERE tasks.task_type = 'GoToPoint'"
    //   );
    //   if(res->rowsCount()!=0){

    //     while(res->next()){
    //       Task task;
    //       task.goal.header.stamp = Util::str_ros_time(res->getString("start_time"));
    //       task.goal.header.frame_id = "map";
    //       task.target_id = res->getInt("target_id");
    //       task.task_id =  res->getInt("task_id");
    //     }
      
    //   }
    
    //}
    
    vector<tuple<int,geometry_msgs::Pose,long double>>
    query_target_pose_and_open_pos_st(string time){
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
      delete res;
      return v;
    }
    
    // get task info to calculate cost
    vector<Task>
    query_runable_tasks(string type){
      sql::ResultSet* res;
      vector<Task> v;
      res = stmt->executeQuery(
       "SELECT * FROM targets tg \
        INNER JOIN tasks ON tasks.target_id = tg.target_id \
        INNER JOIN open_possibilities o WHERE tg.target_id = o.door_id \
        AND DAYOFWEEK(tasks.start_time) = o.day_of_week \
        AND TIME(tasks.start_time) BETWEEN o.start_time AND o.end_time \
        AND tasks.cur_status IN ('Created' , 'WaitingToRun') \
        AND tasks.task_type = 'GatherEnviromentInfo'"
      );
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
        ON tasks.target_id = t.target_id and c.task_id = tasks.task_id"
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
        stmt->executeUpdate("UPDATE costs SET cost = 1.0 * distance + 0.2 * time_diff + (-100) * open_pos_st +(-10) * priority  + (-1.0) * battery");
        res = stmt->executeQuery("SELECT task_id FROM costs WHERE cost = (SELECT min(cost) FROM costs)");
        res->next();
        task_id = res->getInt("task_id"); // get task id which has highest cost
        stmt->executeUpdate("UPDATE tasks SET cur_status = 'Running' WHERE task_id = "+to_string(task_id));
        delete res;
        return task_id;
    }

        // get best task id
    Task query_task_highest_cost(){
        Task task;
        int task_id = -1;
        sql::ResultSet* res;
        stmt->executeUpdate("UPDATE costs SET cost = 1.0 * distance + 0.2 * time_diff + (-100) * open_pos_st +(-10) * priority  + (-1.0) * battery");
    res = stmt->executeQuery("SELECT * FROM costs c INNER JOIN tasks t ON t.task_id = c.task_id WHERE c.cost = (SELECT min(cost) FROM costs)");
        if(res->rowsCount() == 1){
          
          task.task_id = res->getInt("task_id"); // get task id which has highest cost
          task.target_id = res->getInt("target_id");
          task.task_type = res->getString("task_type");
          stmt->executeUpdate("UPDATE tasks SET cur_status = 'Running' WHERE task_id = "+to_string(task_id));
        }
       
        delete res;
        return task;
    }
    
    // Create new enter room tasks
    bool insert_gather_info_tasks(int num, ros::Time start, ros::Duration interval){
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
    
    // Create new charging task and return its task id
    int insert_new_charging_task(int target_id,ros::Time start){
        sql::ResultSet* res;        
        int id = -1;
        stmt->execute(
          "INSERT INTO tasks(task_type, target_id, start_time) VALUES('Charging','" + to_string(target_id) + "','" + Util::time_str(start)+"')"
          );
        res = stmt->executeQuery("SELECT last_insert_id() as id");
        res->next();
        id = res->getInt("id");
        delete res;
        return id;
    }

    int insert_new_go_to_point_task(geometry_msgs::PoseStamped target){
        sql::ResultSet* res;        
        int target_id = -1,task_id = -1;
        stmt->execute(
          "INSERT INTO targets(target_type, position_x, position_y, orientation_z, orientation_w) \
            VALUES('Point'," + to_string(target.pose.position.x) + "," + to_string(target.pose.position.y) + "," + to_string(target.pose.orientation.z) +","+to_string(target.pose.orientation.w)
            +")"
        );

        res = stmt->executeQuery("SELECT last_insert_id() as target_id");
        res->next();
        target_id = res->getInt("target_id");

        stmt->execute(
          "INSERT INTO tasks(task_type, priority, target_id, start_time) VALUES('GoToPoint',4,'" + to_string(target_id) + "','" + Util::time_str(target.header.stamp)+"')"
          );
        res = stmt->executeQuery("SELECT last_insert_id() as task_id");
        res->next();
        task_id = res->getInt("task_id");
        delete res;
        return task_id;
    }

    // Create charging task
    vector<pair<int,geometry_msgs::Pose>>
    query_charging_station(){
      vector<pair<int,geometry_msgs::Pose>> v;
      sql::ResultSet* res;
      res = stmt->executeQuery("SELECT * FROM targets WHERE target_type = 'ChargingStation'");
      while(res->next()){
        geometry_msgs::Pose pose;
        pose.position.x = res->getDouble("position_x");
        pose.position.y = res->getDouble("position_y");
        pose.orientation.z = res->getDouble("orientation_z");
        pose.orientation.w = res->getDouble("orientation_w");
        v.push_back(make_pair(res->getInt("target_id"),pose));
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

    pair<int,string> query_target_id_type_from_task(int task_id){
      sql::ResultSet* res;
      int door;
      string type;
      // get door id
      res = stmt->executeQuery("SELECT * FROM tasks WHERE task_id = "+ to_string(task_id));
      res->next();
      door = res->getInt("target_id");
      type = res->getString("task_type");
      delete res;
      return make_pair(door,type);
    }

    bool insert_record_door_status_list(int door_id, ros::Time measure_time,bool door_status){
      string mst = Util::time_str(measure_time);
       bool result;
      try{
        // insert new record into door status table
        ROS_INFO_STREAM(" insert "<<to_string(door_id)<<" "<<measure_time<<" "<<door_status);
        result = stmt->execute("INSERT INTO door_status(door_id,door_status,date_time) VALUES('" + to_string(door_id) + "', " +to_string(door_status)+", '"+ mst+"')");
        print_table("door_status");
      }catch(sql::SQLException e){
        ROS_INFO_STREAM(e.what());
      }
      return result;
    }

    tuple<string,string,int> query_st_et_dw_from_open_pos(int door_id, ros::Time measure_time){
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
    
    void update_open_pos_table(int door_id, ros::Time measure_time){
      auto t = query_st_et_dw_from_open_pos(door_id,measure_time);

      // update open possibility table
      stmt->executeUpdate(
        "UPDATE open_possibilities o \
          SET  o.open_pos_st = (SELECT SUM(door_status) / COUNT(door_status)  FROM  door_status ds \
              WHERE ds.door_id = '" + to_string(door_id) + "' AND DAYOFWEEK(ds.date_time) = '" + to_string(get<2>(t)) + 
              "' AND TIME(ds.date_time) BETWEEN '" + get<0>(t) + "' AND '"+ get<1>(t) +
          "') WHERE o.door_id = '" + to_string(door_id) + "' AND o.day_of_week = '" + to_string(get<2>(t)) + "' AND o.start_time =' " + get<0>(t) + "' AND o.end_time = '"+ get<1>(t) +"'"       
      );
    }

    void update_task_status(int task_id,string status){
      stmt->executeUpdate("UPDATE tasks set cur_status = '"+ status + "' WHERE task_id = " + to_string(task_id));
    }
    // Change completed task status to 'RanToCompletion'
    void update_task_list_completed(int task_id){
      stmt->executeUpdate("UPDATE tasks set cur_status = 'RanToCompletion' WHERE task_id = " + to_string(task_id));
    }

    // Change cancled task status to 'Canceled'
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
