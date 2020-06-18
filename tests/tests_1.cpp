#include "time_transfer.h"
#include "sql_client.h"
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <thread>
#include <chrono>

class MyTestSuite :public ::testing::Test {
    public:
        MyTestSuite(): sql_client("root","pi"){

        }
        ~MyTestSuite(){}
    TimeTransfer tt;   
    SQLClient sql_client;
};

TEST_F(MyTestSuite,example){
    ASSERT_EQ(1,1);
}

TEST_F(MyTestSuite,transfer_time){
    ASSERT_EQ(ros::Time::now().sec,Util::str_ros_time(Util::time_str(ros::Time::now())).sec);
}

TEST_F(MyTestSuite,insert_task){
    sql_client.insert_new_enter_room_tasks(5,ros::Time::now(),ros::Duration(300));
}

TEST_F(MyTestSuite,insert_tasks_to_cost){
    sql_client.insert_available_task_to_costs(ros::Time::now(),60);
}

// TEST_F(MyTestSuite,query_all_pos_in_cost){
//     auto v = sql_client.query_all_pose_in_costs();
//     ASSERT_GT(v.size(),0);
// }

TEST_F(MyTestSuite,update_distance){
    sql_client.update_distances_in_costs(2,10);
}

TEST_F(MyTestSuite,query_best_task){
    int id = sql_client.query_task_id_highest_cost();
    ASSERT_GT(id,0)<<"id = "<<id;
}

// TEST_F(MyTestSuite,query_tasks){
//     auto task_infos = sql_client.query_all_task_pose_time_open_pos();
//     ASSERT_GT(task_infos.size(),0); 
// }

// TEST_F(MyTestSuite,create_database){
//     std::map<char,geometry_msgs::Pose> map;
//     SQLClient sql_client("root","pi");
//     sql_client.query_multiple_target_position(map,"Door");
//     ASSERT_GT(map.size(),0);
// }

// TEST_F(MyTestSuite,sql_room){
//     std::map<char,geometry_msgs::Pose> map;
//     sql_client.query_rooms_position(map);    
//     ASSERT_GT(map.size(),0);


// TEST_F(MyTestSuite,sql_query_single_room){
//     PossibilityTableRow table_row;
//     DoorStatusListRow list_row;
//     list_row.room_id ='a';
//     list_row.date_time = "2020-06-01 13:30:00";
//     sql_client.query_posibility_table_single_room(table_row,list_row);
//     ASSERT_EQ(table_row.statistuc_open_pos,90);
//     ASSERT_EQ(list_row.door_status,1);
// }

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




int main(int argc, char** argv){
    ros::init(argc,argv,"TestNode");

    ros::NodeHandle nh;

    testing::InitGoogleTest(&argc,argv);

    std::thread t([]{while(ros::ok()) ros::spin();});

    // ::testing::GTEST_FLAG(filter) = "insert_tasks_to_cost*";
    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();
    
    return res;
}
