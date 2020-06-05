#include "ros/ros.h"
#include "robot_navigation/make_task.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <vector>
#include "util.h"
#include "sql_client.h"
#include <string>

#define ROOM_NUM 4
#define TASK_NUM 30
#define DEFAULT_COST 1000
#define SIMULATION_DURATION_SEC 200

typedef struct{
    double distance = 0;
    double sec_diff = 0;
    int periority = 0;
    int    statisic_open_possibility = 0;
    double battery_level = 0;
    double cost;
}CostFunction;

class CentralizedPool{

public:
    CentralizedPool():sql_client(SQLClient::getInstance()){

        init();

        // load room location parameters
        load_room_position();

        // Create available tasks
        create_random_tasks(TASK_NUM,ros::Time::now());


        ros::spin(); // block program
    }

    void init(){

        ROS_INFO_STREAM("Current time: "<<Util::time_str(ros::Time::now()));
	    ros::Duration(1).sleep();
        ROS_INFO_STREAM("Current time: "<<Util::time_str(ros::Time::now()));
        

        // Create a server, usiing make_task.srv file. The service name is make_task
        task_server = nh.advertiseService("make_task",&CentralizedPool::process_robot_request,this);
    
        // Create a client for service "make_plan"
        plan_client = nh.serviceClient<nav_msgs::GetPlan>("move_base/NavfnROS/make_plan");      
    }

    void load_room_position(){
        
        std::string pose_path,ori_path;
        bool ret = false;
        std::vector<double> temp_pos;
        std::vector<double> temp_ori;
        // load task distination
        for(int i = 0; i < ROOM_NUM ; i++){
        
            pose_path = "/room_" + std::string(1,'a' + i)+"_door/position";
            ori_path = "/room_" + std::string(1,'a' + i)+"_door/orientation";
            nh.getParam(pose_path,temp_pos);
            nh.getParam(ori_path,temp_ori);

            geometry_msgs::Pose pose;
            if(temp_pos.size() != 3 || temp_ori.size()!=4){
                ROS_INFO_STREAM("load params failed. position "
                    <<temp_pos.size()<<" orientation "<<temp_ori.size());
                return;
            }
            pose.position.x = temp_pos[0];
            pose.position.y = temp_pos[1];
            pose.position.z = temp_pos[2];

            pose.orientation.x = temp_ori[0];
            pose.orientation.y = temp_ori[1];
            pose.orientation.z = temp_ori[2];
            pose.orientation.w = temp_ori[3];

            room_map.insert(std::pair<char,geometry_msgs::Pose>('a'+i,pose));

        }
        ROS_INFO_STREAM("load "<<room_map.size()<<" positions");
    }


    bool compare_task_cost(const std::pair<EnterRoomTask*,double> &p1, const std::pair<EnterRoomTask*,double> &p2){
        return p1.second>p2.second;
    }

    int get_statistic_open_possibility(char room_id, ros::Time time){
        Table_row table_row;
        List_row list_row;
        table_row.room_id = room_id;
        list_row.room_id = room_id;
        list_row.date_time = Util::time_str(time);
        sql_client.query_posibility_table_single_room(table_row,list_row);
        ROS_INFO_STREAM("room id "<<table_row.room_id<<"pos "<<table_row.statistuc_open_pos);
        return table_row.statistuc_open_pos;
    }
    
    double calculate_cost(EnterRoomTask* task,ros::Time cur_time, geometry_msgs::Pose robot_pos){
            CostFunction cost_function;
            // for each task, calculate time different
            cost_function.sec_diff = (task->goal.header.stamp - cur_time).sec;
            if(cost_function.sec_diff<0){
                ROS_INFO_STREAM("Task "<<task->task_id<<" is expired ");
                return DEFAULT_COST;
            }

            // for each task, check open possibility
            cost_function.statisic_open_possibility = get_statistic_open_possibility(task->room_id,cur_time);
 
            // for each task, request distance from move base plan server
            nav_msgs::GetPlan make_plan_srv;
            make_plan_srv.request.start.pose= robot_pos;
            make_plan_srv.request.start.header = task->goal.header;
            make_plan_srv.request.goal= task->goal;
            make_plan_srv.request.tolerance = 1;
	        
            // request plan with start point and end point
            if(!plan_client.call(make_plan_srv)){
                ROS_INFO_STREAM("Failed to send request to make plan server");
                return DEFAULT_COST;
            }
            // calculate distance
            std::vector<geometry_msgs::PoseStamped> &dists = make_plan_srv.response.plan.poses;

            if(dists.size()==0){
                ROS_DEBUG("Receive empty plan");
                return DEFAULT_COST;
            };   
     
            for(int i = 1; i < dists.size();i++){
                cost_function.distance += sqrt(pow((dists[i].pose.position.x - dists[i-1].pose.position.x),2) + 
                                    pow((dists[i].pose.position.y - dists[i-1].pose.position.y),2));
            }       
             
            cost_function.cost = cost_function.distance + 0.2 * cost_function.sec_diff +
                         100 - cost_function.statisic_open_possibility
                         + cost_function.periority * 5 +  
                         + 100 - cost_function.battery_level;

            ROS_INFO_STREAM("available task room id: "<<task->room_id<<
                                " distance "<<cost_function.distance<<
                                " second different "<<cost_function.sec_diff<<
                                " open possibility "<<cost_function.statisic_open_possibility<<
                                " periority "<< cost_function.periority <<
                                " battery_level "<<cost_function.battery_level <<
                                " cost "<<cost_function.cost);  
            return cost_function.cost;
    }

    bool process_robot_request(robot_navigation::make_task::Request &req,robot_navigation::make_task::Response &res){
        ros::Time cur_time = ros::Time::now();
        int plan_size = 0;
        ROS_INFO_STREAM("receive request from a robot.\n last task: "<<req.last_task<<
                        "\nbettery level "<<req.battery_level<<
                        "\ncurrent position "<<Util::pose_str(req.pose));
        
        if(!req.last_task.is_completed){
            // to do
        }else{
            ROS_INFO_STREAM("Update possibility table");
            List_row list_row;
            list_row.date_time = Util::time_str(cur_time);
            list_row.door_status = req.last_task.door_status;
            list_row.room_id = req.last_task.room_id;
            sql_client.update_possibility_table(list_row);
        }

        ROS_INFO_STREAM("There are "<<cost_vector.size()<<" tasks");

        if(!cost_vector.size()){
            ROS_INFO("No available tasks in centralized pool");
            return false;
        }

        for(int i = 0; i < cost_vector.size(); i++){
            // calculate cost for each task
            cost_vector[i].second = calculate_cost(cost_vector[i].first,cur_time,req.pose);
        }
  
        // sort with task cost
        std::sort(cost_vector.begin(),cost_vector.end(),
            [](const std::pair<EnterRoomTask*,double> &p1, const std::pair<EnterRoomTask*,double> &p2){
                return p1.second>p2.second;
            }
        );    
        
        // response best task to robot
        res.best_task.goal = cost_vector.back().first->goal;
        res.best_task.is_completed = false;
        res.best_task.room_id = cost_vector.back().first->room_id;

         ROS_INFO_STREAM("Give robot the best task "<<res.best_task<<" cost "<<cost_vector.back().second);
       
        // remove this task
        cost_vector.pop_back();
 
        return true;
    }

    void create_random_tasks(int num,ros::Time start_time){
        ROS_INFO_STREAM("start create "<<num<<" tasks");
        int cnt = 0;
        long increase_sec;
        int output_size = 100;
        char output[output_size];
        std::string format = "%Y-%m-%d %H:%M:%S"; // time format
        std::time_t  raw_time;
        ros::Time time;
        
        while(cnt < num){
            increase_sec = rand()%SIMULATION_DURATION_SEC;
            
            // Create a random time after start time 
            time = start_time + ros::Duration(increase_sec);
            
            raw_time = static_cast<time_t>(time.sec); // convert ros time to time_t
            struct tm* time_info = localtime(&raw_time);

          // if (time_info->tm_wday > 0 && time_info->tm_wday < 6 && time_info->tm_hour > 9 && time_info->tm_hour < 20) {
			    
                // Create a task on Monday to Friday from 9 am to 20 pm
                EnterRoomTask* task = new EnterRoomTask();
                task->goal.header.frame_id = "map";
                task->goal.header.stamp = time;  // set task time
                task->priority = rand()%4 + 1;   // set random task priority 1-5
                task->room_id = rand()%ROOM_NUM + 'a';
                task->task_id = rand()%100;
                task->goal.pose=room_map[task->room_id];                
                
                // add this task in tasks vector
                cost_vector.push_back(std::pair<EnterRoomTask*,double>(task,DEFAULT_COST));
                std::strftime(output,output_size,format.c_str(),time_info);
                ROS_INFO_STREAM("Create new Task: "<<output<<" room "<<task->room_id<<" priority "<<task->priority <<" goal "<<task->goal);
		cnt++;
//           }  
        }
    }
    
private:
    ros::ServiceServer task_server;

    ros::ServiceClient plan_client;

    SQLClient sql_client;

    ros::NodeHandle nh;

    std::map<char,geometry_msgs::Pose> room_map;

    std::vector<std::pair<EnterRoomTask*,double>> cost_vector;
};

int main(int argc, char** argv){
    ros::init(argc,argv,"centralized_poor");

    CentralizedPool pool;

    SQLClient sql_client;
    
}
