#include "sql_client.h"
#include "util.h"
#include <gtest/gtest.h>

class SqlTaskTest :public ::testing::Test {
    public:
        SqlTaskTest() {
            sc = new SQLClient("sql_test_1","pass");
            ros::Time now;
            now.sec = 1590994800;
            ros::Time::setNow(now);
        }
    SQLClient* sc;
};



TEST_F(SqlTaskTest,InsertATargetAssignId){
    geometry_msgs::PoseStamped goal;
    int id = sc->InsertATargetAssignId(goal,"Point");
    ASSERT_GT(id,0);
}

TEST_F(SqlTaskTest,InsertATaskAssignId){
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
 //   ASSERT_EQ(sc->UpdateAllPointRoomId(),1);  
    ASSERT_GT(t.point.pointId,0);
    ASSERT_GT( t.taskId,0);
}


TEST_F(SqlTaskTest,QueryRunableExecuteTasks){
    SmallExecuteTask t;
    t.point.pointId = 22;
    t.taskType = "ExecuteTask";
    t.priority = 4;
    t.point.goal.header.stamp = ros::Time::now()+ros::Duration(20);
    t.point.goal.header.frame_id = "map";
    t.taskId = sc->InsertATaskAssignId(t);
    auto v = sc->QueryRunableExecuteTasks(1);
    ASSERT_GT(v.size(),0);
}

// TEST_F(SqlTaskTest,QueryTaskWeight){
//    auto tw = sc->QueryTaskWeight();
//    ASSERT_EQ(tw.wt_btr,1);
//    ASSERT_EQ(tw.wt_wait,1);
//    ASSERT_EQ(tw.wt_psb,-1);
//    ASSERT_EQ(tw.wt_pri,-1);
// } 


TEST_F(SqlTaskTest,TaskUpdateStatus){
    SmallExecuteTask t;
    t.point.pointId = 3;
    t.taskType = "GatherEnviromentInfo";
    t.taskId = sc->InsertATaskAssignId(t); 
    int ret = sc->UpdateTaskStatus(t.taskId,"Running");

    ASSERT_EQ(ret,1);
}


TEST_F(SqlTaskTest,UpdateTaskEndTime){
    SmallExecuteTask t;
    t.point.pointId = 10;
    t.taskType = "ExecuteTask";
    t.taskId = sc->InsertATaskAssignId(t); 
    int ret = sc->UpdateTaskEndTime(t.taskId);
    ASSERT_EQ(ret,1);
}

TEST_F(SqlTaskTest,TaskUpdateDescription){
    SmallExecuteTask t;
    t.point.pointId = 5;
    t.taskType = "GatherEnviromentInfo";
    t.taskId = sc->InsertATaskAssignId(t); 
    int ret = sc->UpdateTaskDescription(t.taskId,"Succeeded");

    ASSERT_EQ(ret,1);
}

TEST_F(SqlTaskTest,QueryRunableChargingTask){
    SmallTask t = sc->QueryRunableChargingTask(0);
    // ASSERT_EQ(t.taskId,1);
}

TEST_F(SqlTaskTest,CallNewExpProcedure){
    sc->CallNewExpProcedure(1);

}
