#pragma once
#include "sql_client.h"
#include "util.h"

class SqlTest :public ::testing::Test {
    public:
        SqlTest() {

        }
        ~SqlTest(){}
    
};

TEST_F(SqlTest,example){
    ASSERT_EQ(1,1);
}


TEST_F(SqlTest,QueryRealTimeDoorInfo){
    SQLClient sql_client("root","nes");
    ros::Time now;
    now.sec = 1590994800;
    ros::Time::setNow(now);
    ASSERT_EQ(ros::Time::now().sec,1590994800);
    auto doors = sql_client.QueryRealTimeDoorInfo();
    ASSERT_EQ(doors.size(),16);
    ASSERT_EQ(doors[12].doorId,13);
    ASSERT_EQ(doors[12].pose.position.x,5.8);
    ASSERT_EQ(doors[12].pose.position.y,7.7);
    ASSERT_EQ(doors[12].depOpenpossibility,0.2);
}


TEST_F(SqlTest,InsertATargetAssignId){
    SQLClient sql_client("root","nes");
    geometry_msgs::PoseStamped goal;
    int id = sql_client.InsertATargetAssignId(goal,"Point");
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
    t.targetId = sql_client.InsertATargetAssignId(t.goal,"Point");
    int taskId = sql_client.InsertATaskAssignId(t);  
    ASSERT_GT(t.targetId,0);
    ASSERT_GT( taskId,0);
}
// // 
// TEST_F(SqlTest,QueryRunableTask){
//      SQLClient sql_client("root","nes");
//     auto v = sql_client.QueryRunableGatherEnviromentInfoTasks();
//     ASSERT_GE(v.size(),0);
// // ASSERT_LT(v.back().goal.header.stamp,ros::Time::now());
//     auto v2 = sql_client.QueryRunableExecuteTasks();
//     ASSERT_GE(v2.size(),0);
// }

TEST_F(SqlTest,InserDoorStatusRecord){
    SQLClient sql_client("root","nes");
    ros::Time now = ros::Time::now();
    int r = sql_client.InsertDoorStatusRecord(1,now,1); 
    // int u = sql_client.UpdateOpenPossibilities(1,now);
    ASSERT_EQ(r,1);
    // ASSERT_EQ(u,1);

}


// // TEST_F(SqlTest,UpdateExpiredTask){
// // }


TEST_F(SqlTest,QueryAvailableChargingStations){
    SQLClient sql_client("root","nes");
    auto v = sql_client.QueryAvailableChargingStations();
    ASSERT_EQ(v.size(),3);
}



TEST_F(SqlTest,UpdateReturnedTask){
    SQLClient sql_client("root","nes");
    sql_client.UpdateReturnedTask(3);
}

// // // TEST_F(SqlTest,create_database){
// // //     std::map<int,geometry_msgs::Pose> map;
// // //     SQLClient sql_client("root","pi");
// // //     sql_client.query_multiple_target_position(map,"Door");
// // //     ASSERT_GT(map.size(),0);
// // // }

// // // TEST_F(SqlTest,insertRecord){   
// // //     ros::Time now = ros::Time::now();
// // //     auto p = sql_client.query_targetId_type_from_task(2);
// // //     ASSERT_EQ(p.first,'b');
// // //     ASSERT_EQ(p.second,"EnterRoom");
// // //     sql_client.InsertDoorStatusRecord(p.first,now,0);
// // //     auto t = sql_client.QueryStartTimeEndTimeDayFromOpenPossibilitiesTable(p.first,ros::Time::now());
// // //     // ASSERT_EQ(get<0>(t),"16:00:00");
// // //     // ASSERT_EQ(get<1>(t),"23:59:59");
// // //     ASSERT_EQ(get<2>(t),5);
// // //     //sql_client.UpdateOpenPossibilities(targetId,now);
// // // }

// // TEST_F(SqlTest,UpdateTaskStatus){
// //     // sql_client.UpdateTaskStatus(1,"Canceled");
// // }

// // TEST_F(SqlTest,insert_to_door_status){
// //     ros::Time now = ros::Time::now();
// //     // sql_client.InsertDoorStatusRecord('a',now,false);
    
// // }
                                                                                                                                                                     

