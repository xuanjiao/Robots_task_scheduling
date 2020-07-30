#pragma once

#include <geometry_msgs/PoseStamped.h>

typedef struct {
    int taskId;
    int priority;
    std::string taskType;
    int targetId;
    geometry_msgs::PoseStamped goal; // distination and timestamp
}Task;