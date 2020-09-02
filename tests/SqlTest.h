#pragma once
#include "../include/sql_client.h"
#include "util.h"
#include <gtest/gtest.h>

class SqlTest :public ::testing::Test {
    public:
        SqlTest() {
            sc = new SQLClient("root","nes");
        }
    SQLClient* sc;
};

TEST_F(SqlTest,example){
    ASSERT_EQ(1,1);
}


TEST_F(SqlTest,QueryDoorInfo){
    ros::Time now;
    now.sec = 1590994800;
    ros::Time::setNow(now);
    ASSERT_EQ(ros::Time::now().sec,1590994800);
    auto doors = sc->QueryDoorInfo();
    ASSERT_EQ(doors.size(),16);
    ASSERT_EQ(doors[12].doorId,13);
    ASSERT_EQ(doors[12].pose.position.x,5.8);
    ASSERT_EQ(doors[12].pose.position.y,7.7);
    ASSERT_GT(doors[12].depOpenpossibility,0.0);
    // ASSERT_EQ(doors[12].isUsed,false);
}

TEST_F(SqlTest,QueryRunableExecuteTasks){
    auto v = sc->QueryRunableExecuteTasks();
    ASSERT_GE(v.size(),0);
}

TEST_F(SqlTest,InsertATargetAssignId){
    geometry_msgs::PoseStamped goal;
    int id = sc->InsertATargetAssignId(goal,"Point");
    ASSERT_GT(id,0);
}
TEST_F(SqlTest,InsertATaskAssignId){
        SQLClient sql_client("root","nes"); 
    SmallExecuteTask t;
    t.taskType = "ExecuteTask";
    t.priority = 4;
    t.point.goal.pose.position.x = 4.38;
    t.point.goal.pose.position.y =  9.34;
    t.point.goal.header.stamp = ros::Time::now()+ros::Duration(20);
    t.point.goal.header.frame_id = "map";
    t.point.pointId = sc->InsertATargetAssignId(t.point.goal,"Point");
    t.taskId = sc->InsertATaskAssignId(t);  
    ASSERT_GT(t.point.pointId,0);
    ASSERT_GT( t.taskId,0);
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
       // ros::Duration(2).sleep();
    auto cs2 = sc->QueryChargingStationInfo(17);
    ASSERT_LE(cs2.remainingTime,100);
    ASSERT_LE(cs2.batteryLevel,100);
}

TEST_F(SqlTest,UpdateChargingStationInfo){
    ChargingStation cs;
    cs.stationId = 18;
    cs.batteryLevel = 65.2;
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
    t.point.pointId = 3;
    t.taskType = "GatherEnviromentInfo";
    t.taskId = sc->InsertATaskAssignId(t); 
    int ret = sc->UpdateTaskStatus(t.taskId,"Running");
    ASSERT_EQ(ret,1);
    auto v = sc->QueryDoorInfo();
    ASSERT_EQ(v[2].isUsed,1);
}

TEST_F(SqlTest,TaskUpdateStatus){
    SmallExecuteTask t;
    t.point.pointId = 3;
    t.taskType = "GatherEnviromentInfo";
    t.taskId = sc->InsertATaskAssignId(t); 
    int ret = sc->UpdateTaskStatus(t.taskId,"Running");

    ASSERT_EQ(ret,1);
}

TEST_F(SqlTest,TaskUpdateDescription){
    SmallExecuteTask t;
    t.point.pointId = 5;
    t.taskType = "GatherEnviromentInfo";
    t.taskId = sc->InsertATaskAssignId(t); 
    int ret = sc->UpdateTaskDescription(t.taskId,"Succeeded");

    ASSERT_EQ(ret,1);
}


// // TEST_F(SqlTest,insert_to_door_status){
// //     ros::Time now = ros::Time::now();
// //     // sc->InsertDoorStatusRecord('a',now,false);
    
// // }
                                                                                                                                                                     

