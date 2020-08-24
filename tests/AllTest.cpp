#include <ros/ros.h>
#include <gtest/gtest.h>
#include "SqlTest.h"
#include "TaskManagerTest.h"
#include <thread>
class Example :public ::testing::Test{
    public: 
    Example(){

    }
};

TEST_F(Example,example){
    ASSERT_EQ(1,1);
}

int main(int argc, char** argv){
    testing::InitGoogleTest(&argc,argv);
    ros::init(argc,argv,"TestNode");

    // ros::NodeHandle nh;

    std::thread t([]{while(ros::ok()) ros::spin();});

    // ::testing::GTEST_FLAG(filter) = "SqlTest.insert_task";
    //  ::testing::GTEST_FLAG(filter) = "SqlTest.InserDoorStatusRecord";

    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();
    
    return res;
}             