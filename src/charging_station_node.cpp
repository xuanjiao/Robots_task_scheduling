#include "ros/ros.h"
#include "sql_client.h"
#include "charging_station.h"
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
    _as(_nh,"charging_action_"+ to_string(stationId),
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

        int ret = _sc.UpdateChargingStationInfo(cs1);
        if(ret != 0 ){
            ROS_INFO("Update station %d succedded",_id);
            cs2 = _sc.QueryChargingStationInfo(_id);
            ros::Duration sleep(cs2.remainingTime);
            sleep.sleep();
            cs3 = _sc.QueryChargingStationInfo(_id);
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