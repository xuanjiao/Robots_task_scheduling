#include "sql_client.h"
#include "util.h"
#include <gtest/gtest.h>

class SqlTasktest :public ::testing::Test {
    public:
        SqlTasktest() {
            sc = new SQLClient("sql_test_1","pass");
        }
    SQLClient* sc;
};


TEST_F(SqlTasktest,QueryRunableExecuteTasks){
    auto v = sc->QueryRunableExecuteTasks(1);
    ASSERT_GT(v.size(),0);
}

TEST_F(SqlTasktest,InsertATargetAssignId){
    geometry_msgs::PoseStamped goal;
    int id = sc->InsertATargetAssignId(goal,"Point");
    ASSERT_GT(id,0);
}

TEST_F(SqlTasktest,InsertATaskAssignId){
        SQLClient sql_client("root","nes"); 
    SmallExecuteTask t;
    t.taskType = "ExecuteTask";
    t.priority = 4;
    t.point.goal.pose.position.x = 4.38;
    t.point.goal.pose.position.y =  9.34;
    t.point.goal.header.stamp = ros::Time::now()+ros::Duration(20);
    t.point.goal.header.frame_id = "map";
    t.point.pointId = sc->InsertATargetAssignId(t.point.goal,"Point");
    t.taskId = sc->InsertATaskAssignId(t);  
    ASSERT_GT(t.point.pointId,0);
    ASSERT_GT( t.taskId,0);
}


TEST_F(SqlTasktest,QueryTaskWeight){
   auto tw = sc->QueryTaskWeight();
   ASSERT_EQ(tw.wt_btr,1);
   ASSERT_EQ(tw.wt_wait,1);
   ASSERT_EQ(tw.wt_psb,-1);
   ASSERT_EQ(tw.wt_pri,-1);
} 


TEST_F(SqlTasktest,TaskUpdateStatus){
    SmallExecuteTask t;
    t.point.pointId = 3;
    t.taskType = "GatherEnviromentInfo";
    t.taskId = sc->InsertATaskAssignId(t); 
    int ret = sc->UpdateTaskStatus(t.taskId,"Running");

    ASSERT_EQ(ret,1);
}


TEST_F(SqlTasktest,TaskUpdateDescription){
    SmallExecuteTask t;
    t.point.pointId = 5;
    t.taskType = "GatherEnviromentInfo";
    t.taskId = sc->InsertATaskAssignId(t); 
    int ret = sc->UpdateTaskDescription(t.taskId,"Succeeded");

    ASSERT_EQ(ret,1);
}

