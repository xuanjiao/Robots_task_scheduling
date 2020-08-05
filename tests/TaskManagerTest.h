#pragma once
#include "ros/ros.h"
#include "TaskManager.h"
#include "sql_client.h"

class TaskManagerTest :public ::testing::Test {
    public:
        TaskManagerTest(): sc("root","nes"),tm(sc,nh){

        }
        ~TaskManagerTest(){}
    SQLClient sc;
    TaskManager tm;
    ros::NodeHandle nh;
};

TEST_F(TaskManagerTest,example){
    ASSERT_EQ(1,1);
}

TEST_F(TaskManagerTest,createTasks){
     tm.CreateNewTasks(10);
}