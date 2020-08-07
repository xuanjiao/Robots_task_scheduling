#pragma once
#include <iostream>
#include <stdlib.h>
#include <geometry_msgs/PoseStamped.h>

typedef struct {
    int taskId = 0;
    int dependency = 0;
    int robotId = 0;
    std::string taskType = "";
    int targetId = 0;
    double openPossibility = 0.0;
    int priority = 0;
    geometry_msgs::PoseStamped goal; // distination and timestamp
    double cost = 0.0;

}TaskInTable;

typedef struct TaskNode{
    int taskId = 0;
    TaskNode* nextNode;
    geometry_msgs::PoseStamped goal;
    double openPossibility = 0.0;
    double cost = 0.0;
}TaskNode;




