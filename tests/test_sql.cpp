#include "time_transfer.h"
#include "sql_client.h"
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include <tuple>

class SqlTest :public ::testing::Test {
    public:
        SqlTest(): sql_client("root","pi"){

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

TEST_F(SqlTest,insert_task){
    sql_client.insert_new_enter_room_tasks(5,ros::Time::now(),ros::Duration(300));
}

TEST_F(SqlTest,set_expired_task_to_canceled){
    sql_client.update_expired_tasks_canceled(ros::Time::now());
}


TEST_F(SqlTest,insert_tasks_to_cost){
    sql_client.insert_available_task_to_costs(ros::Time::now(),5);
}

TEST_F(SqlTest,query_charging_station){
    auto v = sql_client.query_charging_station();
    ASSERT_EQ(v.size(),3);
}

TEST_F(SqlTest,insert_charging_task){
    int id = sql_client.insert_new_charging_task('w',ros::Time::now());
    ASSERT_GT(id,0);
}

TEST_F(SqlTest,update_distance){
    sql_client.update_distances_in_costs(2,10);
}

TEST_F(SqlTest,query_best_task){
    int id = sql_client.query_task_id_highest_cost();
    ASSERT_GT(id,0)<<"id = "<<id;
}

TEST_F(SqlTest,update_returned_task){
    sql_client.update_returned_task(2,ros::Duration(70),2);
    sql_client.update_returned_task(4,ros::Duration(70),4);
}


// TEST_F(SqlTest,query_tasks){
//     auto task_infos = sql_client.query_all_task_pose_time_open_pos();
//     ASSERT_GT(task_infos.size(),0); 
// }

TEST_F(SqlTest,create_database){
    std::map<int,geometry_msgs::Pose> map;
    SQLClient sql_client("root","pi");
    sql_client.query_multiple_target_position(map,"Door");
    ASSERT_GT(map.size(),0);
}

// TEST_F(MyTestSuite, time_increase){
//     TimeTransfer tt;  
//     ASSERT_LT(tt.convert_to_office_time(ros::Time::now()),
//                 tt.convert_to_office_time(ros::Time::now()+ros::Duration(1)));              
// }

// TEST_F(MyTestSuite, sec_0){
//      ASSERT_EQ(tt.convert_to_office_time_string(ros::Time(0)),"2020-06-01 06:00:00"); 
// }

// TEST_F(MyTestSuite, sec_59){
//     ASSERT_EQ(tt.convert_to_office_time_string(ros::Time(59)),"2020-06-01 17:48:00");
// }

// TEST_F(MyTestSuite, sec_60){
//     ASSERT_EQ(tt.convert_to_office_time_string(ros::Time(60)),"2020-06-02 06:00:00");
// }

// TEST_F(MyTestSuite, sec_100){
//     ASSERT_EQ(tt.convert_to_office_time_string(ros::Time(100)),"2020-06-02 14:00:00");
// }

TEST_F(SqlTest,update_pos_table){   
    ros::Time now = ros::Time::now();
    sql_client.update_task_list_completed(2);
    auto p = sql_client.query_target_id_type_from_task(2);
    ASSERT_EQ(p.first,'b');
    ASSERT_EQ(p.second,"EnterRoom");
    sql_client.insert_record_door_status_list(p.first,now,0);
    auto t = sql_client.query_st_et_dw_from_open_pos(p.first,ros::Time::now());
    // ASSERT_EQ(get<0>(t),"16:00:00");
    // ASSERT_EQ(get<1>(t),"23:59:59");
    ASSERT_EQ(get<2>(t),5);
    //sql_client.update_open_pos_table(target_id,now);
}

TEST_F(SqlTest,update_task_status){
    sql_client.update_task_status(1,"Canceled");
}

TEST_F(SqlTest,insert_to_door_status){
    ros::Time now = ros::Time::now();
    sql_client.insert_record_door_status_list('a',now,false);
    
}

int main(int argc, char** argv){
    ros::init(argc,argv,"TestNode");

    ros::NodeHandle nh;

    testing::InitGoogleTest(&argc,argv);

    std::thread t([]{while(ros::ok()) ros::spin();});

    ::testing::GTEST_FLAG(filter) = "SqlTest.insert_task";
    //  ::testing::GTEST_FLAG(filter) = "SqlTest.update_pos_table";
    // ::testing::GTEST_FLAG(filter) = "SqlTest.insert_to_door_status";
    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();
    
    return res;
}
