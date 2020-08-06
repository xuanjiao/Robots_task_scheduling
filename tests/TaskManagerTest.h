#pragma once
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
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

TEST_F(TaskManagerTest,calculateCost){
     vector<TaskInTable> v = sc.QueryRunableGatherEnviromentInfoTasks();
     geometry_msgs::Pose robotPose;
     auto v2 = tm.calculateCostofTasks(v,robotPose);
     ASSERT_LT(v2.size(),5);
     ASSERT_EQ(v.size(),v2.size());
}