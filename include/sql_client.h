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
#include "objects.h"
#include "cost_function.h"
#include "util.h"
#include <boost/thread/mutex.hpp>

#define   DATABASE_NAME                                   "origin_db"
#define   URI                                             "tcp://127.0.0.1"

using namespace std;

class SQLClient{
  public:
  static const int EXP_BEGIN = 1;
  int _exp_id = EXP_BEGIN;

  SQLClient(){}
  SQLClient(string user_name, string pass);
  void ConnectToDatabase(string user_name, string pass);
  void TruncateTable(string name);
  void PrintTable(string table_name);
  TaskWeight QueryTaskWeight();
  DoorWeight QueryDoorWeight();
  vector<tuple<int,geometry_msgs::Pose,long double>> QueryTargetPositionAndOpenPossibilities(string time);
  vector<SmallExecuteTask> QueryRunableExecuteTasks(int robotId);
  SmallTask QueryRunableChargingTask(int robotId);
  int QueryRoomWithCoordinate(geometry_msgs::Pose robotPose);
  vector<Door> QueryDoorInfo();
  vector<double> QueryRelativeDoorOpenPossibility(set<int>& doors, ros::Duration waitingTime );
  ChargingStation QueryChargingStationInfo(int stationId);
  vector<ChargingStation> QueryChargingStationInfo();
  int InsertATaskAssignId(SmallExecuteTask& t);
  int InsertATaskAssignId(SmallTask& t);
  int InsertATargetAssignId(geometry_msgs::PoseStamped target, string targetType);
  int UpdateFailedExecuteTask(const vector<int>& taskIds);
  int InsertDoorStatusRecord(int door_id, ros::Time measure_time,bool door_status);
  int UpdateOpenPossibilities(int door_id, ros::Time measure_time);
  int UpdateTaskStatus(int taskId,string status);
  int UpdateTaskDescription(int taskId, string description);
  int UpdateTaskRobotId(int taskId, int robotId);
  int UpdateChargingStationInfo(const ChargingStation& cs);
  int UpdateTaskEndTime(int taskId);
  bool CallNewExpProcedure();
  bool CallNewEnvExpProcedure();
  void CallFinishExpProcedure();
  ~SQLClient(){ delete stmt; delete _con;}
  private:
  sql::Driver* _driver;
  sql::Connection* _con;
  sql::Statement* stmt;
  boost::mutex _sqlMtx;
};
