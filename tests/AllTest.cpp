#include <ros/ros.h>
#include <gtest/gtest.h>
#include "SqlTest.h"
// #include "TaskManagerTest.h"
#include <thread>

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