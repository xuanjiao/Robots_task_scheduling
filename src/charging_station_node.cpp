#include "ros/ros.h"
#include "sql_client.h"
#include "objects.h"
#include "robot_navigation/ChargingAction.h"
#include <actionlib/server/simple_action_server.h>
#include <vector>

using namespace std;

class ChargingStationController{
    typedef actionlib::SimpleActionServer<robot_navigation::ChargingAction> ActionServer;
public:
    ChargingStationController(int stationId):
    _id(stationId),
    _sc("charging_station","pass"),
    _as(_nh,"/charging_action_"+ to_string(stationId),
        boost::bind(&ChargingStationController::ExecuteCallback,this,_1),false)
    {
        initActionServer();
    }

    void initActionServer(){
        ROS_INFO("Charging station %d start",_id); 
        _as.start();
    }

    void ExecuteCallback(const robot_navigation::ChargingGoalConstPtr &robot){
        robot_navigation::ChargingResult rs;
        ChargingStation cs1,cs2,cs3;
        ROS_INFO("Station %d: Start charging for robot %d (%d)...",_id,robot->robotId,robot->battery);
        cs1.batteryLevel = robot->battery;
        cs1.stationId = _id;
        cs1.robotId = robot->robotId;

        int ret = _sc.UpdateChargingStationInfo(cs1);
        if(ret == 0 ){
            ROS_INFO("Update station %d failed",_id);
            _as.setAborted(rs);
            return;
        }

        ROS_INFO("Update station %d succedded",_id);
        
        ros::Rate loop(1);
        while(ros::ok()){ // Query charging station status every 1s
            cs2 = _sc.QueryChargingStationInfo(_id);
            if(cs2.cur_status == "Next_exp"){
                ROS_INFO("Charging finished");
                _as.setSucceeded(rs);
                break;        
            }else{
                ROS_INFO("Robot %d battery %.2f remain time  %.2f: ",cs2.robotId, cs2.batteryLevel,cs2.remainingTime);
            }
            ros::spinOnce();
            loop.sleep();
        }
        cs3.robotId = 0;
        cs3.stationId = _id;
        _sc.UpdateChargingStationInfo(cs3);
    }

    ~ChargingStationController(){
    }


    int _id;
    SQLClient _sc;   
    ros::NodeHandle _nh;
    ActionServer _as;

};

int main(int argc,char** argv){
    ros::init(argc,argv,"charging_station"); // Initializes Node Name
    if(argc != 2){
        ROS_INFO("Total %d arguments. Require 1 arg: station id ",argc);
        return 1;
    }
    int stationId = atoi(argv[1]);

    ROS_INFO("Charging Station Controller %d run",stationId);
    ChargingStationController csc(stationId);
    ros::spin();
}


/*
void ExecuteCallback(const robot_navigation::ChargingGoalConstPtr &robot){
        robot_navigation::ChargingResult rs;
        ChargingStation cs1,cs2,cs3;
        ROS_INFO("Station %d: Start charging for robot %d (%d)...",_id,robot->robotId,robot->battery);
        cs1.batteryLevel = robot->battery;
        cs1.stationId = _id;
        cs1.robotId = robot->robotId;

        int ret = _sc.UpdateChargingStationInfo(cs1);
        if(ret != 0 ){
            ROS_INFO("Update station %d succedded",_id);
            ros::Duration(1).sleep();
            cs2 = _sc.QueryChargingStationInfo(_id);
            ros::Duration sleep(cs2.remainingTime + 1);
            sleep.sleep();
            cs3 = _sc.QueryChargingStationInfo(_id);
            ROS_INFO("Charging for %.2f second finished. Current level %.2f: ",cs2.remainingTime, cs3.batteryLevel);
            if(cs3.batteryLevel == 100){
                _as.setSucceeded(rs);
            }else{
                _as.setAborted(rs);
            }
        }else{
            ROS_INFO("Update station %d failed",_id);
            _as.setAborted(rs);
        }

    }
*/