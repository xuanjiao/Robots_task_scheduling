# robot_navigation
## How to use this program

### 1.  Create log file of door sensor:
1. install mysql library
'''
sudo apt-get install libmysqlcppconn-dev
'''

2. log in mysql server 

    `mysql -r root -p`

3. create tables
```
    source /home/[user_name]/catkin_ws/src/robot_navigation/sql/run.sql
```
![database](./img/robot-database.png)

targets
![targets](./img/targets.png)

open_possibilities
![open_pos](./img/open_possibilities.png)

tasks
![tasks](./img/tasks.png)

cost
![costs](./img/costs.png)

### 2.  Start sensor node:
```
    roslaunch robot_navigation office_world.launch
    rosrun rosrun robot_navigation door_status_advertiser
```
## 3. Start navigation stack
```
    roslaunch robot_navigation robot_navigation.launch
```
use estimate position tool in rviz to estimate position

## 4. run demo
```
    roslaunch robot_navigation move_demo.launch
```
## Structure

![structure](./img/robot-ros_structure.png)

## work flow
![work flow](./img/robot-ros_workflow.png)

cost_function.cost = 1.0 * distance + 0.2 * sec_diff + (-1.0) * statisic_open_possibility +(-10) * priority  +  (-1.0) * battery_level;

robot choose task with lowest cost

## Problem 

battery module

Option 

1.  Gazebo plug in 
    
    https://github.com/pooyanjamshidi/brass_gazebo_battery
    
2. Gazebo official battery class

    http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1common_1_1Battery.html

3.   Calculate battery consume use angle of rotation and distance

## TO DO

- simulation time issue
- robot battery consume
- continue sql part: update table with robor result
- read paper

