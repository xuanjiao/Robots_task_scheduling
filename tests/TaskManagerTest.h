#pragma once
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include "task_manager.h"
#include "sql_client.h"
#include "general_task.h"

using namespace std;

class TaskManagerTest :public ::testing::Test {
    public:
        TaskManagerTest()
        : 
        sc("root","nes")
        // ,tm(sc,nh)
        {

        }
        ~TaskManagerTest(){}
    SQLClient sc;
    ros::NodeHandle nh;
};

TEST_F(TaskManagerTest,example){
    ASSERT_EQ(1,1);
}

TEST_F(TaskManagerTest,createTasks){
    TaskManager tm(sc,nh);
     tm.LoadMapInfo();
     tm.CreateNewTasks(10);
}


TEST_F(TaskManagerTest,createSeries){
    TaskManager tm(sc,nh);
    std::vector<TaskInTable> tasks;
    TaskInTable t1;
    t1.taskId = 1;
    t1.goal.pose.position.x = 2;
    t1.goal.pose.orientation.w = 2;
    t1.dependency = 0;
    TaskInTable t2;
    t2.taskId = 2;
    t2.goal.pose.position.x = 4;
    t2.dependency = 1;
    TaskInTable t3;
    t3.taskId = 3;
    t3.goal.pose.position.x = 8;
    t3.dependency = 0;
    tasks.push_back(t1);
    tasks.push_back(t2);
    tasks.push_back(t3);
    auto lts = tm.MakeLargeTasks(tasks);
    ASSERT_EQ(lts.size(),2);
    ASSERT_EQ(lts[0].tasks.size(),2);
    ASSERT_EQ(lts[1].tasks.size(),1);

    ASSERT_EQ(lts[0].tasks[1].pose.position.x,t1.goal.pose.position.x);
    ASSERT_EQ(lts[0].tasks[2].pose.position.x,t2.goal.pose.position.x);
    ASSERT_EQ(lts[1].tasks[3].pose.position.x,t3.goal.pose.position.x);
    // ASSERT_EQ(lts[1].tasks[3],t3.goal);

    // geometry_msgs::Pose robotPose;
    // robotPose.position.x = 3;
    // robotPose.orientation.w = 1;
    // tm.CalculateCostForSerie(lts,robotPose);
    // ASSERT_EQ(lts[0][0].cost,0);
    // ASSERT_EQ(lts[0][1].cost,0);
    // ASSERT_EQ(lts[1][0].cost,0);

}

// TEST_F(TaskManagerTest,calculateBattery){
//     TaskManager tm(sc,nh);
//     geometry_msgs::Pose p1,p2,p3;
//     geometry_msgs::PoseStamped ps2,ps3;
//     p1.position.x = 0.0; p1.position.y = 5.0; p1.orientation.w = 1.0;
//     p2.position.x = 2.0; p2.position.y = 5.0; p2.orientation.w = 1.0;
//     p3.position.x = 5.0; p3.position.y = 5.0; p3.orientation.w = 1.0;
//     ps2.pose = p2;ps3.pose = p3;
//     std::map<int,geometry_msgs::PoseStamped> map;
//     map.insert(make_pair(2,ps2));
//     map.insert(make_pair(3,ps3));
//     double d12 = tm.CalculatSmallTaskBatteryConsumption(p1,p2);
//     double d23 = tm.CalculatSmallTaskBatteryConsumption(p2,p3);
//     double d13 = tm.CalculatSmallTaskBatteryConsumption(p1,p3);
//     ASSERT_EQ(d13,d12 + d23);
//     double d = tm.CalculateLargeTaskBatteryConsumption(p1,map);
//     ASSERT_EQ(d13,d);

// }