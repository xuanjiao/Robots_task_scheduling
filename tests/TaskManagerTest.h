#pragma once
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include "task_manager.h"
#include "sql_client.h"
#include "general_task.h"

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
    // ASSERT_EQ(1,1);
}

// TEST_F(TaskManagerTest,createTasks){
//      tm.CreateNewTasks(10);
// }

// TEST_F(TaskManagerTest,calculateCost){
//      vector<TaskInTable> v = sc.QueryRunableGatherEnviromentInfoTasks();
//      geometry_msgs::Pose robotPose;
//      auto v2 = tm.CalculateCostofTasks(v,robotPose);
//      ASSERT_LT(v2.size(),5);
//      ASSERT_EQ(v.size(),v2.size());
// }

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
    t2.dependency = 1;
    TaskInTable t3;
    t3.taskId = 3;
    t3.dependency = 0;
    tasks.push_back(t1);
    tasks.push_back(t2);
    tasks.push_back(t3);
    auto series = tm.MakeTaskSerie(tasks);
    ASSERT_EQ(series.size(),2);
    ASSERT_EQ(series[0][0].taskId,1);
    ASSERT_EQ(series[1][0].taskId,3);

    geometry_msgs::Pose robotPose;
    robotPose.position.x = 3;
    robotPose.orientation.w = 1;
    tm.CalculateCostForSerie(series,robotPose);
    ASSERT_EQ(series[0][0].cost,0);
    ASSERT_EQ(series[0][1].cost,0);
    ASSERT_EQ(series[1][0].cost,0);

}