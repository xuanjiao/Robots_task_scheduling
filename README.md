# Master Thesis: Exploiting Knowledge of Room Occupation for the Sche- duling of Navigation Tasks of a Fleet of Robots in Office Environments (2020)

# Table of content
-   Introduction
    -   Background
    -   Goad
-   Installation
    -   Install MySQL database
    -   Install ROS
    -   Download project folder
-   Usage
-   Project directory

# Introduction

## Background
Robots has many potential in office environment. They allow people to communicate without direct social contact, which is very popular in the current lock-down situation. For example, robots can deliver documents, or find colleagues to start video conferences. However, compared to the office building, the robot is small, and its respective is limited. If robots want to schedule their activities properly according to their surrounding environment, they need additional information source, sensor modules. 

## Goals
Given a multi-room office environment that has sensors, robots, and a task server called centralized pool. 

1. Find a way to gather information about room occupation effectively. 
2. Use information from robots and environment (e.g. room occupation) to schedule tasks, reducing total energy and time. 

# Installation
### Install MySQL database
1. Install mysql library

     `sudo apt-get install libmysqlcppconn-dev`

2. Log in mysql server 

    `mysql -r root -p`

3. Use SQL schema to create tables

    `source /home/[user_name]/catkin_ws/src/robot_navigation/sql/run.sql`

### Install ROS
    ROS tutorial 

    https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/

### Download project folder

    Download this project to home/catkin_ws
    
    git clone https://github.com/xuanjiao/robots_task_scheduling.git

# Usages
1.  Start Gazebo simulator

    `roslaunch turtlebot3_gazebo multi_turtlebot3.launch`

2.  Start navigation stack

    ` roslaunch turtlebot3_gazebo multi.launch`

3.  Start ROS nodes: robots, centralized pool, charging station, sensor simulator, etc.

    `roslaunch robot_navigation move_demo.launch`

# Project directory
```
├── action/
│   ├── Charging.action     % Charging Action provided by charging stations 
│   └── RunTask.action      % RunTask Action provided by robots
├── CMakeLists.txt      
├── include/
│   ├── cost_function.h     % Cost calculation class
│   ├── objects.h           % Target class: doors, points and charging stations
│   ├── room_map.h          % map information
│   ├── sql_client.h        % MySQL Client class
│   ├── task_manager.h      % Task scheduling class
│   ├── task_type.h         % Task definition
│   └── util.h              % Some help function
├── launch/
│   ├── gtest.test          % Run unit test
│   └── move_demo.launch    % Launch ROS nodes: robots, centralized pool, charging station, sensor simulator
├── maps/
│   └── office.pgm          % Map file used by the map server
├── msg/
│   ├── sensor_data.msg     % Sensor message   
├── README.md
├── sql/                    % Database statements
│   ├── create_execute_task.sql
│   ├── create_possibility_table.sql
│   ├── createRawData.sql
│   ├── experiment/
│   │   ├── charging_event.sql
│   │   ├── next_env_exp.sql
│   │   ├── next_exe_exp.sql
│   │   ├── update_env_exp_result.sql
│   │   └── update_exe_exp_result.sql
│   ├── run.sql
├── src/
│   ├── centralized_pool_node.cpp   % Centralized pool
│   ├── charging_station_node.cpp   % Charging station
│   ├── door_status_node.cpp        % Door sensor
│   ├── robot_controller_node.cpp   % Robot
├── srv/
│   └── GetATask.srv        % Task service provided by the centralized pool
├── tests/                  % Unit tests
│   ├── AllTest.cpp
│   ├── other_test.h
│   ├── sql_object_test.h
│   ├── sql_task_test.h
│   └── TaskManagerTest.h
└── world/
    ├── office.world        % Simulator configuration
```
