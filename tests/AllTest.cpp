#include <ros/ros.h>
#include <gtest/gtest.h>
#include "TaskManagerTest.h"
#include "sql_object_test.h"
#include "sql_task_test.h"
#include "other_test.h"
#include <thread>

int main(int argc, char** argv){
    testing::InitGoogleTest(&argc,argv);
    ros::init(argc,argv,"TestNode");

    // ros::NodeHandle nh;

    std::thread t([]{while(ros::ok()) ros::spin();});

    // ::testing::GTEST_FLAG(filter) = "SqlTest.insert_task";
    ::testing::GTEST_FLAG(filter) = "SqlTaskTest.CallNewExpProcedure";

    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();
    
    return res;
}             