#include "time_transfer.h"
#include "sql_client.h"
#include <ros/ros.h>

#include <gtest/gtest.h>

#include <thread>
#include <chrono>

class MyTestSuite :public ::testing::Test {
    public:
        MyTestSuite(){

        }
        ~MyTestSuite(){}
    TimeTransfer tt;
    SQLClient sql_client;
};

TEST_F(MyTestSuite,sql_room){
    std::map<char,geometry_msgs::Pose> map;
    sql_client.query_rooms_position(map);    
    ASSERT_GT(map.size(),0);
}

TEST_F(MyTestSuite,sql_charging_station){
    std::map<char,geometry_msgs::Pose> map;
    sql_client.query_charging_stations_position(map);
    ASSERT_GT(map.size(),0);
}

TEST_F(MyTestSuite, time_increase){
    ASSERT_LT(tt.convert_to_office_time(ros::Time::now()),
                tt.convert_to_office_time(ros::Time::now()+ros::Duration(1)));              
}

TEST_F(MyTestSuite, sec_0){
     ASSERT_EQ(tt.convert_to_office_time_string(ros::Time(0)),"2020-06-01 06:00:00"); 
}

TEST_F(MyTestSuite, sec_59){
    ASSERT_EQ(tt.convert_to_office_time_string(ros::Time(59)),"2020-06-01 17:48:00");
}

TEST_F(MyTestSuite, sec_60){
    ASSERT_EQ(tt.convert_to_office_time_string(ros::Time(60)),"2020-06-02 06:00:00");
}

TEST_F(MyTestSuite, sec_100){
    ASSERT_EQ(tt.convert_to_office_time_string(ros::Time(100)),"2020-06-02 14:00:00");
}


int main(int argc, char** argv){
    ros::init(argc,argv,"TestNode");

    ros::NodeHandle nh;

    testing::InitGoogleTest(&argc,argv);

    std::thread t([]{while(ros::ok()) ros::spin();});

    auto res = RUN_ALL_TESTS();

    ros::shutdown();
    
    return res;
}
