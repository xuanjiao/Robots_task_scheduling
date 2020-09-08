#pragma once

#include <gtest/gtest.h>
#include <actionlib/TestAction.h>
#include <actionlib/client/simple_action_client.h>
#include "robot_navigation/ChargingAction.h"

using namespace actionlib;

class ServiceTest :public ::testing::Test {
    public:
        ServiceTest() {
            
        }
        ~ServiceTest(){}
};

TEST_F(ServiceTest,clientSendGoalAndWaitTest){
    // ros::NodeHandle n;
    // SimpleActionClient<robot_navigation::ChargingAction> client(n, "/charging_action_17");
    // robot_navigation::ChargingGoal goal;
    // goal.battery = 50;
    // goal.robotId = 1;
    // actionlib::SimpleClientGoalState state(actionlib::SimpleClientGoalState::LOST);

    // bool started = client.waitForServer(ros::Duration(10.0));
    // ASSERT_TRUE(started);

    // state = client.sendGoalAndWait(goal,ros::Duration(10,0),ros::Duration(10,0));
    
    // ASSERT_EQ(state,SimpleClientGoalState::SUCCEEDED);
}