#include <ros/ros.h>
#include <gtest/gtest.h>
#include "sql_test.h"
#include "object_test.h"
#include "TaskManagerTest.h"
#include "service_test.h"
#include <thread>

int main(int argc, char** argv){
    testing::InitGoogleTest(&argc,argv);
    ros::init(argc,argv,"TestNode");

    // ros::NodeHandle nh;

    std::thread t([]{while(ros::ok()) ros::spin();});

    // ::testing::GTEST_FLAG(filter) = "SqlTest.insert_task";
    //  ::testing::GTEST_FLAG(filter) = "SqlTest.UpdateTaskStatus";

    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();
    
    return res;
}             