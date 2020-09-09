#pragma once

#include <gtest/gtest.h>
#include "room_map.h"
using namespace std;

class OtherTest :public ::testing::Test {
    public:
        OtherTest() {
            
        }
        ~OtherTest(){}
};

TEST_F(OtherTest,getRelativeDoors){
    auto doors = RoomMap::getRelativeDoors(1,8);
    ASSERT_EQ(doors.size(),3);
    ASSERT_TRUE(doors.count(1));
    ASSERT_TRUE(doors.count(7));
    ASSERT_TRUE(doors.count(8));
}