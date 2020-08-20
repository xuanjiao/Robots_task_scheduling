#pragma once
#include "general_task.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include "ros/ros.h"
#include "door_info.h"

using namespace std;

struct TaskWeightBase{
    double W_BATTERY       = 10;
    double W_TIME          = 0.1;
    double W_POSSIBILITY   = 10;
    double W_PRIORITY      = 10;
}TWB;

struct DoorWeightBase{
    double W_TIME          = 0.1;
    double W_BATTERY       = 10;
    double W_POSSIBILITY   = 10;
}DWB;

class CostCalculator{
    public:

    CostCalculator(ros::NodeHandle &nh):_nh(nh){
        _pc = _nh.serviceClient<nav_msgs::GetPlan>("/tb3_0/move_base/NavfnROS/make_plan"); 
    }

    void CalculateLargeTasksCost(ros::Time now,LargeTask& t, geometry_msgs::Pose robotPose){
        t.battery = CalculateComplexTrajectoryBatteryConsumption(robotPose,t.tasks);
        t.waitingTime = t.tasks.begin()->second.header.stamp - now; 
        t.cost =  TWB.W_BATTERY/ t.tasks.size() * t.battery 
                    + TWB.W_TIME  * t.waitingTime.toSec() 
                    + TWB.W_POSSIBILITY * t.openPossibility 
                    + TWB.W_PRIORITY * t.priority;
        ROS_INFO("%d        %.3f   %.3f   %.3f  %d  %3f",t.largeTaskId,t.battery,t.waitingTime.toSec(), t.openPossibility,t.priority,t.cost);

    }

    void CalculateDoorCost(ros::Time now, Door& door, geometry_msgs::Pose robotPose){
        double battery =  CalculateSimpleBatteryConsumption(robotPose,door.pose);
        double timeSinceLastUpdate = (now - door.lastUpdate).sec;      
        door.cost = DWB.W_BATTERY * battery + DWB.W_POSSIBILITY * door.depOpenpossibility * DWB.W_TIME * timeSinceLastUpdate;
    }


    double CalculateComplexTrajectoryBatteryConsumption(geometry_msgs::Pose robotPose,std::map<int,geometry_msgs::PoseStamped> points){
        double battery = 0.0;
        geometry_msgs::Pose start;
        map<int,geometry_msgs::PoseStamped>::iterator it = points.begin();
        battery += CalculateSimpleBatteryConsumption(robotPose,it->second.pose);
        start = it->second.pose;
        for( it++;it != points.end();it++){
          battery += CalculateSimpleBatteryConsumption(start,it->second.pose);
          start = it->second.pose;
        }
        return battery;
    }

    double CalculateSimpleBatteryConsumption(geometry_msgs::Pose start, geometry_msgs::Pose end){
            double distance = 0,angle = 0,batteryConsumption = 0;
        // for each task, request distance from move base plan server
            nav_msgs::GetPlan make_plan_srv;
            make_plan_srv.request.start.pose= start;
            make_plan_srv.request.start.header.frame_id = "map";
            make_plan_srv.request.goal.pose = end;
            make_plan_srv.request.goal.header.frame_id = "map";
            make_plan_srv.request.tolerance = 1;
	        
            // request plan with start point and end point
            if(!_pc.call(make_plan_srv)){
                ROS_INFO_STREAM("Failed to send request to make plan server");
                return -1;
            }
            // calculate distance
            std::vector<geometry_msgs::PoseStamped> &dists = make_plan_srv.response.plan.poses;

            if(dists.size()==0){
                ROS_DEBUG("Receive empty plan");
                return -1;
            };   
     
            for(size_t i = 1; i < dists.size();i++){
                distance = sqrt(pow((dists[i].pose.position.x - dists[i-1].pose.position.x),2) + 
                                    pow((dists[i].pose.position.y - dists[i-1].pose.position.y),2));
                                     
                angle = 2 * acos(dists[i].pose.orientation.w);
                batteryConsumption = batteryConsumption + 0.01 * distance + 0.001 * angle;
                // ROS_INFO("From (%.3f,%.3f) to (%.3f,%) linear variation = %.3f, angle variation = %.3f, battery consum = %.3f",
                        // dists[i-1].pose.position.x,dists[i-1].pose.position.y,dists[i].pose.position.x,dists[i].pose.position.y,distance,angle,batteryConsumption);
            }   
            return batteryConsumption;   
    }

private:
    ros::ServiceClient _pc;
    ros::NodeHandle _nh;
};
// llcf = {10,10/ntaks,0.1,-10,-1)}