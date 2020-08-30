#pragma once
#include "../include/sql_client.h"
#include "charging_station.h"
#include "util.h"
#include <gtest/gtest.h>

class SqlTest :public ::testing::Test {
    public:
        SqlTest() {
            sc = new SQLClient("root","nes");
        }
        ~SqlTest(){}
    SQLClient* sc;
};

TEST_F(SqlTest,example){
    ASSERT_EQ(1,1);
}


TEST_F(SqlTest,QueryDoorInfo){
    SQLClient sql_client("root","nes");
    ros::Time now;
    now.sec = 1590994800;
    ros::Time::setNow(now);
    ASSERT_EQ(ros::Time::now().sec,1590994800);
    auto doors = sc->QueryDoorInfo();
    ASSERT_EQ(doors.size(),16);
    ASSERT_EQ(doors[12].doorId,13);
    ASSERT_EQ(doors[12].pose.position.x,5.8);
    ASSERT_EQ(doors[12].pose.position.y,7.7);
    ASSERT_EQ(doors[12].depOpenpossibility,0.2);
    // ASSERT_EQ(doors[12].isUsed,false);
}


TEST_F(SqlTest,InsertATargetAssignId){
    SQLClient sql_client("root","nes");
    geometry_msgs::PoseStamped goal;
    int id = sc->InsertATargetAssignId(goal,"Point");
    ASSERT_GT(id,0);
}
TEST_F(SqlTest,InsertATaskAssignId){
        SQLClient sql_client("root","nes"); 
    SmallExecuteTask t;
    t.taskType = "ExecuteTask";
    t.priority = 4;
    t.goal.pose.position.x = 4.38077210276;
    t.goal.pose.position.y =  9.34650744461;
    t.goal.header.stamp = ros::Time::now()+ros::Duration(20);
    t.goal.header.frame_id = "map";
    t.targetId = sc->InsertATargetAssignId(t.goal,"Point");
    int taskId = sc->InsertATaskAssignId(t);  
    ASSERT_GT(t.targetId,0);
    ASSERT_GT( taskId,0);
}


TEST_F(SqlTest,InserDoorStatusRecord){
    SQLClient sql_client("root","nes");
    ros::Time now = ros::Time::now();
    int r = sc->InsertDoorStatusRecord(1,now,1); 
    // int u = sc->UpdateOpenPossibilities(1,now);
    ASSERT_EQ(r,1);
    // ASSERT_EQ(u,1);

}

TEST_F(SqlTest,QueryChargingStationInfo){
    SQLClient sql_client("root","nes");
    auto cs = sc->QueryChargingStationInfo(17);
    ASSERT_EQ(cs.remainingTime,0);
    ASSERT_EQ(cs.batteryLevel,100);
    ros::Duration(1).sleep();
    vector<ChargingStation> v = sc->QueryChargingStationInfo();
    ASSERT_EQ(v.size(),2);
    ASSERT_GT(v[0].remainingTime,0);
    ASSERT_LT(v[0].batteryLevel,100);

}

TEST_F(SqlTest,UpdateChargingStationInfo){
    ChargingStation cs;
    cs.stationId = 18;
    cs.batteryLevel = 10;
    int ret = sc->UpdateChargingStationInfo(cs);
    ASSERT_EQ(ret,1);
}




// // // TEST_F(SqlTest,create_database){
// // //     std::map<int,geometry_msgs::Pose> map;
// // //     SQLClient sql_client("root","pi");
// // //     sc->query_multiple_target_position(map,"Door");
// // //     ASSERT_GT(map.size(),0);
// // // }



TEST_F(SqlTest,TaskInUseTrigger){
    SmallExecuteTask t;
    t.targetId = 3;
    t.taskType = "GatherEnviromentInfo";
    t.taskId = sc->InsertATaskAssignId(t); 
    int ret = sc->UpdateTaskStatus(t.taskId,"Running");
    ASSERT_EQ(ret,1);
    auto v = sc->QueryDoorInfo();
    ASSERT_EQ(v[2].isUsed,1);
}

TEST_F(SqlTest,TaskUpdateStatus){
    SmallExecuteTask t;
    t.targetId = 3;
    t.taskType = "GatherEnviromentInfo";
    t.taskId = sc->InsertATaskAssignId(t); 
    int ret = sc->UpdateTaskStatus(t.taskId,"Running");

    ASSERT_EQ(ret,1);
}

TEST_F(SqlTest,TaskUpdateDescription){
    SmallExecuteTask t;
    t.targetId = 5;
    t.taskType = "GatherEnviromentInfo";
    t.taskId = sc->InsertATaskAssignId(t); 
    int ret = sc->UpdateTaskDescription(t.taskId,"Succeeded");

    ASSERT_EQ(ret,1);
}


// // TEST_F(SqlTest,insert_to_door_status){
// //     ros::Time now = ros::Time::now();
// //     // sc->InsertDoorStatusRecord('a',now,false);
    
// // }
                                                                                                                                                                     

