#pragma once

#include <geometry_msgs/PoseStamped.h>

typedef struct {
    int task_id;
    int priority;
    std::string task_type;
    int target_id;
    geometry_msgs::PoseStamped goal; // distination and timestamp
}Task;