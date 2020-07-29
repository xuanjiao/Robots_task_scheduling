#include "time_transfer.h"
#include "sql_client.h"
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include <tuple>

class SqlTest :public ::testing::Test {
    public:
        SqlTest(): sql_client("root","nes"){

        }
        ~SqlTest(){}
    TimeTransfer tt;   
    SQLClient sql_client;
};

TEST_F(SqlTest,example){
    ASSERT_EQ(1,1);
}

// TEST_F(MyTestSuite,transfer_time){
//     ASSERT_EQ(ros::Time::now().sec,Util::str_ros_time(Util::time_str(ros::Time::now())).sec);
// }

// TEST_F(SqlTest,insert_task){
//     sql_client.InsertMultipleGatherInfoTasks(5,ros::Time::now(),ros::Duration(300));
// }

// TEST_F(SqlTest,InsertATargetAssignId){
//     geometry_msgs::PoseStamped goal;
//     int id = sql_client.InsertATargetAssignId(goal);
//     ASSERT_GT(id,0);
// }

TEST_F(SqlTest,InsertATaskAssignId){
           
    Task t;
    t.task_type = "ExecuteTask";
    t.priority = 4;
    t.goal.pose.position.x = 4.38077210276;
    t.goal.pose.position.y =  9.34650744461;
    t.goal.pose.orientation.z = 0.721488227349;
    t.goal.pose.orientation.w = 0.692426702112;
    t.goal.header.stamp = ros::Time::now()+ros::Duration(20);
    t.goal.header.frame_id = "map";
    t.target_id = sql_client.InsertATargetAssignId(t.goal,"Point");
    int taskId = sql_client.InsertATaskAssignId(t);  
    ASSERT_GT(t.target_id,0);
    ASSERT_GT( taskId,0);
}

TEST_F(SqlTest,QueryRunableTask){
    auto v = sql_client.QueryRunableGatherEnviromentInfoTasks();
    // ASSERT_GT(v.size(),0);
// ASSERT_LT(v.back().goal.header.stamp,ros::Time::now());
    auto v2 = sql_client.QueryRunableExecuteTasks();
    ASSERT_GT(v2.size(),0);
}


TEST_F(SqlTest,UpdateExpiredTask){
}


TEST_F(SqlTest,QueryAvailableChargingStations){
    auto v = sql_client.QueryAvailableChargingStations();
    ASSERT_EQ(v.size(),3);
}



TEST_F(SqlTest,UpdateReturnedTask){
    sql_client.UpdateReturnedTask(1,3,ros::Duration(2000));
}

// TEST_F(SqlTest,create_database){
//     std::map<int,geometry_msgs::Pose> map;
//     SQLClient sql_client("root","pi");
//     sql_client.query_multiple_target_position(map,"Door");
//     ASSERT_GT(map.size(),0);
// }

// TEST_F(SqlTest,update_pos_table){   
//     ros::Time now = ros::Time::now();
//     auto p = sql_client.query_target_id_type_from_task(2);
//     ASSERT_EQ(p.first,'b');
//     ASSERT_EQ(p.second,"EnterRoom");
//     sql_client.InsertDoorStatusRecord(p.first,now,0);
//     auto t = sql_client.QueryStartTimeEndTimeDayFromOpenPossibilitiesTable(p.first,ros::Time::now());
//     // ASSERT_EQ(get<0>(t),"16:00:00");
//     // ASSERT_EQ(get<1>(t),"23:59:59");
//     ASSERT_EQ(get<2>(t),5);
//     //sql_client.UpdateOpenPossibilities(target_id,now);
// }

TEST_F(SqlTest,UpdateTaskStatus){
    // sql_client.UpdateTaskStatus(1,"Canceled");
}

TEST_F(SqlTest,insert_to_door_status){
    ros::Time now = ros::Time::now();
    // sql_client.InsertDoorStatusRecord('a',now,false);
    
}

int main(int argc, char** argv){
    testing::InitGoogleTest(&argc,argv);
    ros::init(argc,argv,"TestNode");

    ros::NodeHandle nh;

    std::thread t([]{while(ros::ok()) ros::spin();});

    // ::testing::GTEST_FLAG(filter) = "SqlTest.insert_task";
    //  ::testing::GTEST_FLAG(filter) = "SqlTest.UpdateExpiredTask";

    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();
    
    return res;
}                                                                                                                                                                                  

