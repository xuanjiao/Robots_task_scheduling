# Master Thesis: Exploiting Knowledge of Room Occupation for the Sche- duling of Navigation Tasks of a Fleet of Robots in Office Environments (2020)

This is a robot system that can schedule robot tasks according to environmental information including room occupation.

# Prerequisite
[Ubuntu 16.04](https://ubuntu.com/tutorials/install-ubuntu-desktop-1604#1-overview)

[ROS](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

[ROS simulation packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)

[MySQL Workbench](https://dev.mysql.com/downloads/workbench/)

[MySQL Connector/C++](https://dev.mysql.com/doc/dev/connector-cpp/8.0/)
# Installation

1. Log in mysql server 

`mysql -r root -p`

2. Use SQL schema to create tables

`source /home/[user_name]/catkin_ws/src/robot_navigation/sql/run.sql`

3. Download this project to home/catkin_ws
    
    
```
git clone https://github.com/xuanjiao/robots_task_scheduling.git
```

# Usages
1.  Start Gazebo simulator

`roslaunch turtlebot3_gazebo multi_turtlebot3.launch`

2.  Start navigation stack

` roslaunch turtlebot3_gazebo multi.launch`

3.  Start ROS nodes: robots, centralized pool, charging station, sensor simulator, etc.

`roslaunch robot_navigation move_demo.launch`

# Main features
-   The door sensors can measure and store door status in the local table.
-   The robot is able to request the sensor data (table) as long as it enters the sensor range.
-   The robot then sends acquired room occupation table to the centralized server.
-   The centralized pool make decisions based on information from robot and environment.
# Demo

[Link to Youtube video](https://youtu.be/Y7iX4Zc0Ej4)
