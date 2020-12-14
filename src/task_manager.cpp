#include "task_manager.h"

TaskManager::TaskManager(ros::NodeHandle& nh,SQLClient& sc, CostCalculator& cc):_nh(nh),_sc(sc),_cc(cc){
    
    // dm[Key(0,3)] = {1}; 
    // dm[Key(0,3)] = {1}; 

    // Start timer
    ros::TimerOptions to(
        ros::Duration(ENV_EXP_TIME), // Period
        boost::bind(&TaskManager::FinishEnviromentExperiment,this,_1), // Callback
        NULL, //  global callback queue
        true,  // One shot
        false // Auto start
    );
    _exp_timer = _nh.createTimer(to);
};

void TaskManager::FinishEnviromentExperiment(const ros::TimerEvent& event){
    _sc.CallFinishExpProcedure();
    _exp_timer.stop();
    ROS_INFO("Experiment %d finished",_sc._exp_id);
}



SmallTask TaskManager::GetAChargingTask(int robotId){
    return _sc.QueryRunableChargingTask(robotId);
}

LargeExecuteTask TaskManager::SelectExecutetask(int robotId, geometry_msgs::Pose robotPose){
    vector<SmallExecuteTask> sts;
    vector<LargeExecuteTask> lts;
    LargeExecuteTask lt;
    ros::Time now = ros::Time::now();
    auto w = _sc.QueryTaskWeight();
    _cc.LoadTaskWeight(w);
    sts  = _sc.QueryRunableExecuteTasks(robotId);
    ROS_INFO("Found %ld execute tasks",sts.size());
    // FilterTask(v);
    if(sts.size() != 0 ){  
        lts = LargeExecuteTask::MakeLargeTasks(sts);
            
        // ROS_INFO_STREAM("Large_task_id Battery WaitTime Open_possibility Priority   Cost");
        // ROS_INFO("-----------------------------------------------------------------------------");
        for(LargeExecuteTask& t:lts){
            t.startRoom = _sc.QueryRoomWithCoordinate(robotPose);
            CalculatePossibilityProduct(t);     
            _cc.CalculateLargeTasksCost(now,t,robotPose);
            // ROS_INFO_STREAM("Calculate execute task cost finish");
        } 
        // LargeExecuteTask::FilterTask(lts); // remove task exceed cost limit
    }

    stringstream ss;
    
    if(lts.size() != 0){ // after filter, if there is no execute task, gather inviroment
        //  ss << "After filter, there are tasks: ";
        //  for(LargeExecuteTask& t:lts){
        //     ss << t.taskId <<" ";
        //  } 
        
        LargeExecuteTask::SortTasksWithCost(lts);
        lt = lts.back();
        ss << "the best is "<< lt.taskId;
    }else{
        // ss << "After filter, there are no execute tasks.";
    }
    ROS_INFO_STREAM(ss.str());
    return lt;
}

SmallTask TaskManager::CreateBestEnviromentTask(geometry_msgs::Pose robotPose){
    SmallTask st;
    ros::Time now = ros::Time::now();
    
    auto doors = _sc.QueryDoorInfo();
    ROS_INFO("%ld available doors",doors.size());
    if(doors.empty()){
        ROS_INFO("No door data in database");
        exit(1);
    }   
    DoorWeight dw = _sc.QueryDoorWeight();
    _cc.LoadDoorWeight(dw);
    ROS_INFO_STREAM("Id BatteryComsume TimeSinceLastUpdate Openpossibility Cost");
    ROS_INFO("-----------------------------------------------------------------------------");
    for(Door& door : doors){
            // this door is not exploring by other doors;
            CalculatePossibilityProduct(door,robotPose);
            _cc.CalculateDoorCost(now,door,robotPose);     
        //ROS_INFO("calculate from  (%s) door %d to (%s)",Util::pose_str(robotPose).c_str(), door.doorId, Util::pose_str(door.pose).c_str());
    }
    Door::SortDoorsWithCost(doors);

    ROS_INFO("Best door is %d",doors.back().doorId);
    st.targetId = doors.back().doorId;
    st.goal.pose = doors.back().pose;
    st.goal.header.stamp = now + ros::Duration(TASK_DELAY);
    st.goal.header.frame_id = "map";
    st.taskType = "GatherEnviromentInfo";
    st.priority = 1;
    st.taskId =  _sc.InsertATaskAssignId(st);
    return st;
}

SmallTask TaskManager::CreateBestChargingTask(geometry_msgs::Pose robotPose){
    SmallTask st;
    ros::Time now = ros::Time::now();
    auto css = _sc.QueryChargingStationInfo();
    if(css.empty()){
        ROS_INFO("No door data in database");
        exit(1);
    }
    ROS_INFO_STREAM("Id remaining time battery level  Cost");
    ROS_INFO("-----------------------------------------------------------------------------");
    for(ChargingStation& cs : css){
        _cc.CalculateChargingStationCost(cs,robotPose);
    }
    ChargingStation::SorChargingStationsWithCost(css);

    ROS_INFO("Best charging station is %d",css.back().stationId);
    st.targetId = css.back().stationId;
    st.goal.pose = css.back().pose;
    st.goal.header.stamp = now + ros::Duration(TASK_DELAY);
    st.goal.header.frame_id = "map";
    st.taskType = "Charging";
    st.priority = 5;
    st.taskId =  _sc.InsertATaskAssignId(st);
    return st;
}

void TaskManager::HandleTaskFeedback(TaskFeedback& fb){
    int r = _sc.InsertDoorStatusRecord(fb.doorId,fb.measureTime,fb.doorStatus); 
    int u = _sc.UpdateOpenPossibilities(fb.doorId,fb.measureTime);
    // ROS_INFO("Insert %d record, update %d rows in possibility table",r,u);
}

void TaskManager::HandleTaskResult(TaskResult result){
    if(result.isCompleted){
        for(int id : result.taskIds){
            
            int ret1 = _sc.UpdateTaskStatus(id,"Succedded");
            int ret2 = _sc.UpdateTaskDescription(id,result.description);
            // ROS_INFO("Update task status %d result %d",ret1,ret2);
            if(id == result.taskIds.back()){ // if it is the last task
                int ret3 = _sc.UpdateTaskEndTime(id);
            //     ROS_INFO("Update task end time %d",ret3);
            }
        }
    }else{
        if(result.taskType == "GatherEnviromentInfo"){
            _sc.UpdateTaskStatus(result.taskIds[0],"Error");
            _sc.UpdateTaskDescription(result.taskIds[0],result.description);
        }else if (result.taskType == "Charging"){
            _sc.UpdateTaskStatus(result.taskIds[0],"Canceled");
                _sc.UpdateTaskDescription(result.taskIds[0],result.description);
        }else if(result.taskType == "ExecuteTask"){
            // Change task status from Running to ToReRun, increase priority 3 and increase 60200s start time 
            _sc.UpdateFailedExecuteTask(result.taskIds);
        }else{
            ROS_INFO("Get a unknown task");
        } 
    }

    // When robot 1 finish charging, start next experiment
    if(result.taskType == "Charging" && result.robotId == 1){
        
        
        _sc.CallNewExpProcedure();
        // _sc.CallNewEnvExpProcedure();
        _exp_timer.setPeriod(ros::Duration(ENV_EXP_TIME));
        _exp_timer.start();
    }

}

void TaskManager::AfterSendingTask(int taskId, int robotId){
    _sc.UpdateTaskStatus(taskId,"Running");
    _sc.UpdateTaskRobotId(taskId,robotId);
}

RltDoors TaskManager::CalculatePossibilityProduct(Door& d,geometry_msgs::Pose robotPose){
    RltDoors ds;
    int startRoom = 0, endRoom = 0;
    startRoom = _sc.QueryRoomWithCoordinate(robotPose);
    endRoom = _sc.QueryRoomWithCoordinate(d.pose);   
    d.product_psb = 1;
    if(startRoom != endRoom){
        ds = RoomMap::getRelativeDoors(startRoom,endRoom);
        auto psbs = _sc.QueryRelativeDoorOpenPossibility(ds,ros::Duration(TASK_DELAY));
        for(double psb : psbs){
            d.product_psb *= psb;
        }
    }
    
    return ds;
}

RltDoors TaskManager::CalculatePossibilityProduct(LargeExecuteTask& t){
    RltDoors d,dAll;

    stringstream ss;
    
    auto sit =t.smallTasks.begin();
    int room_2 = sit->second.point.roomId;
    int room_1 = t.startRoom;
    d= RoomMap::getRelativeDoors(room_1,room_2);
        dAll.insert(d.begin(),d.end()); // put relative doors in 
    
    for(sit++; sit !=t.smallTasks.end();sit++){          
        room_1 = room_2; 
        // find relative door for small tasks
        room_2 = sit->second.point.roomId;
        d = RoomMap::getRelativeDoors(room_1,room_2);
        dAll.insert(d.begin(),d.end()); // put relative doors in   
    }
    
    if(dAll.empty()){
        t.openPossibility = 1;
        return dAll;
    }

    // for(auto door: dAll){
    //     ss << door<<" ";
    // }
    // ROS_INFO_STREAM("Large task related door: "<<ss.str());
    // doors.erase(0); // ignore 0
    vector<double> ops =  _sc.QueryRelativeDoorOpenPossibility(dAll,t.waitingTime);
    
    t.openPossibility = 1;
    for(auto op : ops){
        t.openPossibility *= op;
    }

    // ROS_INFO_STREAM(" multiply open possibility "<<t.openPossibility);
    return dAll;
}