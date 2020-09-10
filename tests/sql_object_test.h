#pragma once
#include "sql_client.h"
#include "util.h"
#include <gtest/gtest.h>
#include <geometry_msgs/PoseStamped.h>

class SqlObjectTest :public ::testing::Test {
    public:
        SqlObjectTest() {
            sc = new SQLClient("sql_test_2","pass");
            ros::Time now;
            now.sec = 1590994800;
            ros::Time::setNow(now);
        }
    SQLClient* sc;
};

TEST_F(SqlObjectTest,QueryRoomWithCoordinate){
    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = -4;
  ASSERT_EQ(sc->QueryRoomWithCoordinate(pose),15);

}

TEST_F(SqlObjectTest,QueryDoorInfo){

    //ASSERT_EQ(ros::Time::now().sec,1590994800);
    auto doors = sc->QueryDoorInfo();
    ASSERT_EQ(doors.size(),16);
    ASSERT_EQ(doors[12].doorId,13);
    ASSERT_EQ(doors[12].pose.position.x,5.8);
    ASSERT_EQ(doors[12].pose.position.y,7.7);
    ASSERT_GT(doors[12].depOpenpossibility,0.0);
    // ASSERT_EQ(doors[12].isUsed,false);
}


TEST_F(SqlObjectTest,InserDoorStatusRecord){
    SQLClient sql_client("root","nes");
    ros::Time now = ros::Time::now();
    int r = sc->InsertDoorStatusRecord(1,now,1); 
    // int u = sc->UpdateOpenPossibilities(1,now);
    ASSERT_EQ(r,1);
    // ASSERT_EQ(u,1);

}

TEST_F(SqlObjectTest,QueryChargingStationInfo){
       // ros::Duration(2).sleep();
    auto cs2 = sc->QueryChargingStationInfo(17);
    ASSERT_LE(cs2.remainingTime,100);
    ASSERT_LE(cs2.batteryLevel,100);
}

TEST_F(SqlObjectTest,UpdateChargingStationInfo){
    ChargingStation cs;
    cs.stationId = 18;
    cs.batteryLevel = 65.2;
    int ret = sc->UpdateChargingStationInfo(cs);
    ASSERT_EQ(ret,1);
}

TEST_F(SqlObjectTest, QueryRelativeDoorOpenPossibility){
    set<int> doors;
    doors.insert(3);
    doors.insert(4);
    doors.insert(4);
    doors.insert(5);
    auto o = sc->QueryRelativeDoorOpenPossibility(doors,ros::Duration(100));
    ASSERT_EQ(o.size(),3);
    ASSERT_GT(o[0],0);
    ASSERT_GT(o[1],0);
    ASSERT_GT(o[2],0);


}
  



// // TEST_F(SqlObjectTest,insert_to_door_status){
// //     ros::Time now = ros::Time::now();
// //     // sc->InsertDoorStatusRecord('a',now,false);
    
// // }
                                                                                                                                                                     

