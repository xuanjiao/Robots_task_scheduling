#include "ros/ros.h"
#include "robot_navigation/make_task.h"
#include "robot_navigation/RequestTask.h"
#include "robot_navigation/GoToTargetAction.h"
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>
#include <vector>
#include "util.h"
#include "sql_client.h"
#include <tuple>
#include <string>
#include <queue>

typedef actionlib::SimpleActionClient<robot_navigation::GoToTargetAction> GoToTargetActionClient;                                                                                                                                                                                  ;
typedef struct cost_st{                                                                                                         
    double _distance = 0;
    double _sec_diff = 0;                                           
    int _priority = 0;
    double    _open_pos_st = 0;
    double _battery_level = 0;
    double _cost;
    cost_st(double distance,double sec_diff,double priority,double open_pos_st,double battery_level):
        _distance(distance),_sec_diff(sec_diff),_priority(priority),_open_pos_st(open_pos_st),_battery_level(battery_level){
            _cost = 1.0 * _distance + 0.2 * _sec_diff + (-100) * _open_pos_st +(-10) * _priority  + (-1.0) * _battery_level;
        }
}CostFunction;

class CentralizedPool{

public:
 CentralizedPool():
        _sc("centralized_pool","pass"),
        _gac("GoToTargetActionClient",true) //  spins up a thread to service this action's subscriptions. 
    {
        init();
        create_gather_info_tasks(10);
    }

    void init(){
	    ros::Duration(1).sleep();
        ROS_INFO_STREAM("Current office time: "<<Util::time_str(ros::Time::now()));
        // _ts = _nh.advertiseService("make_task",&CentralizedPool::process_robot_request,this);
        _ts = _nh.advertiseService("RequestTask",&CentralizedPool::when_robot_request_task,this);
        _pc = _nh.serviceClient<nav_msgs::GetPlan>("move_base/NavfnROS/make_plan");      
    
    }

    void create_gather_info_tasks(int num){
        _sc.print_table("targets");
        _sc.truncate_costs_tasks(); // clear task table and cost table
        _sc.insert_gather_info_tasks(num,ros::Time::now(),ros::Duration(30));
        _sc.print_table("tasks");
    }

    void create_go_to_point_task(){
        geometry_msgs::PoseStamped goal;
        goal.pose.position.x = -2;
        goal.pose.position.y = -0.5;
        goal.pose.position.z = -0;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        _sc.insert_new_go_to_point_task(goal);
    }
    
    double calculate_distance(geometry_msgs::Pose target_pose, geometry_msgs::Pose robot_pose){
            double distance = 0;
        // for each task, request distance from move base plan server
            nav_msgs::GetPlan make_plan_srv;
            make_plan_srv.request.start.pose= robot_pose;
            make_plan_srv.request.start.header.frame_id = "map";
            make_plan_srv.request.goal.pose = target_pose;
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
                distance += sqrt(pow((dists[i].pose.position.x - dists[i-1].pose.position.x),2) + 
                                    pow((dists[i].pose.position.y - dists[i-1].pose.position.y),2));
            }   

            return distance;    
    }
    
    Task get_best_task(geometry_msgs::Pose robot_pose,ros::Time t,double battery){
        vector<Task> v;
        _sc.update_expired_tasks_canceled(t); // set exired task to canceled
        if(battery < 20){ // charging
            // create_charging_task(t,robot_pose);
        }else{
            // find if there are execute task    
            v = _sc.query_runable_tasks("ExecuteTask");
            ROS_INFO_STREAM("found "<<v.size()<<" execute tasks");
            if(v.size() ==0){
                v = _sc.query_runable_tasks("GatherEnviromentInfo");
                ROS_INFO_STREAM("found "<<v.size()<<"gather enviroment info tasks");
            }
        }
        if(v.size()==0){
            Task task;
            task.task_id = -1;
            return task;
        }

        ROS_INFO_STREAM("Task_id  Task_type Target_id Priority Open_pos Distance Sec_diff Cost");
        ROS_INFO("-----------------------------------------------------------------------------");
        for(auto &i :v){
               CostFunction cf(
                   calculate_distance(i.goal.pose,robot_pose),
                   i.goal.header.stamp.sec - t.sec,
                   i.priority,
                   i.open_pos,
                   battery
                );
                i.cost = cf._cost;

                ROS_INFO_STREAM(
                    setw(3) <<i.task_id <<" "<<setw(5)<<i.task_type <<setw(4) << i.target_id << setw(2) << i.priority << setprecision(3)<< setw(4) << 
                    i.open_pos << " " << setprecision(3)<< setw(5) << cf._distance << " "<<fixed<<setprecision(3) << setw(10) <<cf._sec_diff<< " " <<fixed << setprecision(3) << setw(6) <<i.cost 
                );
        }

        std::sort(v.begin(),v.end(),
        [](const Task& a, const Task&b)->bool
        {
                return a.cost >b.cost;
        });

        for(auto j:v){
            ROS_INFO_STREAM(j.task_id<<" "<<j.cost);
        }
        return v.back();
    }

    bool process_robot_request(robot_navigation::make_task::Request &req,robot_navigation::make_task::Response &res){
        ros::Time cur_time = ros::Time::now();
        int task_id = req.last_task.task_id;
        ROS_INFO_STREAM("Receive request from robot\n"<<req);
       
        if(task_id == 0){
            ROS_INFO_STREAM("Get initial request from robot");
        }else{
            auto p = _sc.query_target_id_type_from_task(task_id);
            
            // Process enter room task
            if(p.second == "GatherEnviromentInfo"){
                if(!req.last_task.is_completed){
                    ROS_INFO_STREAM("Task failed");
                    _sc.update_task_status(task_id,"Error");
                }else{
                    ROS_INFO_STREAM("Task Succeed");
                    use_task_result(task_id,p.first,req.last_task.m_time, req.last_task.door_status); 
                    if(req.last_task.door_status == false){
                        reuse_task(task_id);
                    }                
                }
            }       
            // Process charging task
            else if(p.second == "Charging"){
                if(!req.last_task.is_completed){
                    ROS_INFO_STREAM("Robot charging failed");
                }else{
                    ROS_INFO_STREAM("Robot charging succedd");
                }
            }
        }

        // Give robot new task 
        Task bt = get_best_task(req.pose,cur_time,req.battery_level);
        _sc.print_table("tasks"); 
        ROS_INFO_STREAM("Best task id = "<<bt.task_id<<" ,cost = "<< fixed << setprecision(3) << setw(6)<< bt.cost);
        res.best_task.task_id = bt.task_id;
        res.best_task.goal = bt.goal; 
        res.best_task.room_id = bt.target_id;
        res.best_task.task_type = bt.task_type;
        
        return true;             
    }

    // call back when receive robot request for task
    bool when_robot_request_task(robot_navigation::RequestTask::Request &req, 
        robot_navigation::RequestTask::Response &res){  
        ROS_INFO_STREAM("Receive request from robot\n"<<req);
        _sc.print_table("tasks"); 
        ros::Time cur_time = ros::Time::now();
        Task bt = get_best_task(req.pose,cur_time,req.battery_level);
        if(bt.task_id == -1){
            res.has_task = false;
        }else{
            ROS_INFO_STREAM("Best task id = "<<bt.task_id<<" ,cost = "<< fixed << setprecision(3) << setw(6)<< bt.cost);
            res.has_task = true;
            send_robot_action(bt); 
        }
    }

    // Send robot new task 
    void send_robot_action(Task &bt){
        robot_navigation::GoToTargetGoal g;
        g.goal = bt.goal;
        g.target_id = bt.target_id;
        g.task_type = bt.task_type;
        g.target_id = bt.target_id;
    }

    void reuse_task(int task_id){
        // change task status from Running to WaitingToRun, increase 3 priority and increase 200s start time 
        _sc.update_returned_task(task_id,ros::Duration(200),3);
    }

    void use_task_result(int task_id,int door_id, ros::Time m_time, bool door_status){
        _sc.update_task_list_completed(task_id); // mark task as RanToCompletion
        _sc.insert_record_door_status_list(door_id,m_time,door_status); 
        _sc.update_open_pos_table(door_id,m_time);
    }

 
    std::pair<int,geometry_msgs::PoseStamped> 
    create_charging_task(ros::Time t,geometry_msgs::Pose rp){
        auto v = _sc.query_charging_station();
        if(v.size()==0){
            ROS_INFO_STREAM("Failed to load charging station");
            exit(1);
        }

        std::pair<int,double> best; // best charging station
        for(auto i : v){
            double dist = calculate_distance(i.second,rp);
            if(dist>best.second){
                best.first = i.first;
                best.second = dist;
            }
        }
        int id = _sc.insert_new_charging_task(best.first,t);

        auto p = find_if( begin(v), end(v),
                             [=](decltype(*begin(v)) item )->int
                             { return get< 0 >( item ) == best.first;} );
        if(p == end(v)) {
            ROS_INFO_STREAM("Failed to find best charging station");
            exit(0);
        }
        geometry_msgs::PoseStamped pst;
        pst.pose = (*p).second;
        pst.header.frame_id = "map";
        pst.header.stamp = t;
        return make_pair(id,pst);
    }

private:
    ros::ServiceServer _ts;
    ros::ServiceClient _pc;
    GoToTargetActionClient _gac;
    SQLClient _sc;
    ros::NodeHandle _nh;
};

int main(int argc, char** argv){
    ros::init(argc,argv,"centralized_poor");
    CentralizedPool pool;
    ros::spin(); // block program
}