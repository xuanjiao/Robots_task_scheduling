#pragma once
#include "sql_client.h"
#include "util.h"

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
    TaskInTable t;
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


// // TEST_F(SqlTest,UpdateExpiredTask){
// // }


// TEST_F(SqlTest,QueryAvailableChargingStations){
//     SQLClient sql_client("root","nes");
//     auto v = sc->QueryAvailableChargingStations();
//     ASSERT_EQ(v.size(),3);
// }




// // // TEST_F(SqlTest,create_database){
// // //     std::map<int,geometry_msgs::Pose> map;
// // //     SQLClient sql_client("root","pi");
// // //     sc->query_multiple_target_position(map,"Door");
// // //     ASSERT_GT(map.size(),0);
// // // }

// // // TEST_F(SqlTest,insertRecord){   
// // //     ros::Time now = ros::Time::now();
// // //     auto p = sc->query_targetId_type_from_task(2);
// // //     ASSERT_EQ(p.first,'b');
// // //     ASSERT_EQ(p.second,"EnterRoom");
// // //     sc->InsertDoorStatusRecord(p.first,now,0);
// // //     auto t = sc->QueryStartTimeEndTimeDayFromOpenPossibilitiesTable(p.first,ros::Time::now());
// // //     // ASSERT_EQ(get<0>(t),"16:00:00");
// // //     // ASSERT_EQ(get<1>(t),"23:59:59");
// // //     ASSERT_EQ(get<2>(t),5);
// // //     //sc->UpdateOpenPossibilities(targetId,now);
// // // }

TEST_F(SqlTest,UpdateTaskStatus){
    sc->UpdateTaskStatus(1,"Canceled");
}

// // TEST_F(SqlTest,insert_to_door_status){
// //     ros::Time now = ros::Time::now();
// //     // sc->InsertDoorStatusRecord('a',now,false);
    
// // }
                                                                                                                                                                     

