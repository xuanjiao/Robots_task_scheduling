#pragma once

#include <gtest/gtest.h>
#include <actionlib/TestAction.h>
#include <objects.h>
using namespace std;
class ObjectTest :public ::testing::Test{
    public: 
    ObjectTest(){

    }
};

TEST_F(ObjectTest,SearchDoorMap){
    ASSERT_EQ(DOORMAP[Key(0,3)].front(),1);
}
