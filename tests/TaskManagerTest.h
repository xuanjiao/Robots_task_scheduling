#pragma once
#include "ros/ros.h"
#include <gtest/gtest.h>
#include <geometry_msgs/PoseStamped.h>
#include "task_manager.h"
#include "sql_client.h"
#include "task_type.h"
#include "objects.h"

using namespace std;

class TaskManagerTest :public ::testing::Test {
    public:
        TaskManagerTest()
        {
            sc= new SQLClient("centralized_pool","pass");
            cc = new CostCalculator(nh);
            tm = new TaskManager(*sc,*cc);
            ros::Time now;
            now.sec = 1590994800;
            ros::Time::setNow(now);
        }
        ~TaskManagerTest(){
            delete sc;
            delete cc;
            delete tm;
        }
    SQLClient *sc;
    TaskManager * tm;
     CostCalculator* cc;
    ros::NodeHandle nh;
};

TEST_F(TaskManagerTest,example){
    ASSERT_EQ(1,1);
}


TEST_F(TaskManagerTest,createLargeTask){
    ASSERT_EQ(ros::Time::now().sec,1590994800);
    std::vector<SmallExecuteTask> tasks;
    SmallExecuteTask t1;
    t1.taskId = 1;
    t1.point.goal.pose.position.x = -24;
    t1.point.goal.pose.position.y = 12;
    t1.point.goal.pose.orientation.w = 2;
    t1.point.roomId = 3; 
    t1.dependency = 0;
    SmallExecuteTask t2;
    t2.taskId = 2;
    t2.point.goal.pose.position.x = 4;
    t2.dependency = 1;
    SmallExecuteTask t3;
    t3.taskId = 3;
    t3.point.goal.pose.position.x = 8;
    t3.dependency = 0;
    tasks.push_back(t1);
    tasks.push_back(t2);
    tasks.push_back(t3);
    auto lts = LargeExecuteTask::MakeLargeTasks(tasks);
    ASSERT_EQ(lts.size(),2);
    ASSERT_EQ(lts[0].smallTasks.size(),2);
    ASSERT_EQ(lts[1].smallTasks.size(),1);

    ASSERT_EQ(lts[0].smallTasks[1].point.goal.pose.position.x,t1.point.goal.pose.position.x);
    ASSERT_EQ(lts[0].smallTasks[2].point.goal.pose.position.x,t2.point.goal.pose.position.x);
    ASSERT_EQ(lts[1].smallTasks[3].point.goal.pose.position.x,t3.point.goal.pose.position.x);
}

TEST_F(TaskManagerTest,HandleFailedTaskResult){
    TaskResult r;
    r.isCompleted = false;
    r.taskIds.push_back(1);
    r.description = "test";
    r.taskType = "ExecuteTask";
    tm->HandleTaskResult(r);
}

TEST_F(TaskManagerTest,HandleSuccessTaskResult){
    TaskResult r;
    r.isCompleted = true;
    r.taskIds.push_back(17);
    r.taskIds.push_back(18);
    r.description = "test";
    r.taskType = "ExecuteTask";
    tm->HandleTaskResult(r);
}


TEST_F(TaskManagerTest,HandleMultipleLatgeTaskResult){
    TaskResult r1;
    r1.isCompleted = true;
    r1.taskIds.push_back(27);;
    r1.description = "test";
    r1.taskType = "ExecuteTask";

        TaskResult r2;
    r2.isCompleted = true;
    r2.taskIds.push_back(28);;
    r2.description = "test";
    r2.taskType = "ExecuteTask";

        TaskResult r3;
    r3.isCompleted = true;
    r3.taskIds.push_back(29);;
    r3.description = "test";
    r3.taskType = "ExecuteTask";
    tm->HandleTaskResult(r1);
    tm->HandleTaskResult(r2);
    tm->HandleTaskResult(r3);
}

TEST_F(TaskManagerTest,CalculateLargeTaskPossibilityProduct){
    
    LargeExecuteTask lt;
    lt.waitingTime = ros::Duration(20);
    lt.startRoom = 1;
    SmallExecuteTask s1,s2; // corridor -> room 3 -> room 4
    s1.point.roomId = 3;
    s2.point.roomId = 4;
    lt.smallTasks.insert(make_pair(1,s1));
    lt.smallTasks.insert(make_pair(2,s2));
    tm->CalculatePossibilityProduct(lt);

    ASSERT_LT(lt.openPossibility - 0.16, 0.01); // 0.8 x 0.2

    LargeExecuteTask lt2;  // room 1 -> point 6 -> point 7
    SmallExecuteTask s3,s4;
    lt2.startRoom = 1;
    s3.point.roomId = 12;
    s4.point.roomId = 13;
    lt2.smallTasks.insert(make_pair(6,s3)); 
    lt2.smallTasks.insert(make_pair(7,s4));
    auto s = tm->CalculatePossibilityProduct(lt2);
    ASSERT_EQ(s.count(1),1);// check if contains door 1,10,12,13
    ASSERT_EQ(s.count(10),1);
    ASSERT_EQ(s.count(12),1);
    ASSERT_EQ(s.count(13),1);

    ASSERT_LT(lt2.openPossibility - 0.4096, 0.01); // 0.8 x 0.8 x 0.8 x 0.8

}

TEST_F(TaskManagerTest,CalculateEnviromentPossibilityProduct){
    Door door;
    geometry_msgs::Pose robotPose;
    robotPose.position.x = 3.5;
    robotPose.position.y = 7.7;
    door.pose.position.x = -16.0;
    door.pose.position.y = 7.7;
    auto doors = tm->CalculatePossibilityProduct(door,robotPose);
    ASSERT_EQ(doors.count(1),1);
    ASSERT_EQ(doors.size(),2);
    ASSERT_EQ(doors.count(10),1);
    ASSERT_LT(door.product_psb - 0.64, 0.01); // 0.8 x 0.8
}

// TEST_F(TaskManagerTest,calculateBattery){
//     geometry_msgs::Pose p1,p2,p3;
//     geometry_msgs::PoseStamped ps2,ps3;
//     p1.position.x = 0.0; p1.position.y = 5.0; p1.orientation.w = 1.0;
//     p2.position.x = 2.0; p2.position.y = 5.0; p2.orientation.w = 1.0;
//     p3.position.x = 5.0; p3.position.y = 5.0; p3.orientation.w = 1.0;
//     ps2.pose = p2;ps3.pose = p3;
//     std::map<int,geometry_msgs::PoseStamped> map;
//     map.insert(make_pair(2,ps2));
//     map.insert(make_pair(3,ps3));
//     double d12 = tm->CalculateSimpleBatteryConsumption(p1,p2);
//     double d23 = tm->CalculateSimpleBatteryConsumption(p2,p3);
//     double d13 = tm->CalculateSimpleBatteryConsumption(p1,p3);
//     ASSERT_EQ(d13,d12 + d23);
//     double d = tm->CalculateComplexTrajectoryBatteryConsumption(p1,map);
//     ASSERT_EQ(d13,d);

// }