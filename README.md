# robot_navigation
## How to use this program

### 1.  Create log file of door sensor:
1. log in mysql server 

    `mysql -r root -p`

2. create door open percent table 
```
    source /home/[user_name]/catkin_ws/src/robot_navigation/sql/createDoorOpenPercentTable.sql
```
![input](./img/possibility_input.png)
createDoorOpenPercentTable.sql

![occupancy table](img/occupation_posibility_table.png)
program result

3. create a log file for door sensor in room a-d
```
    source /home/[user_name]/catkin_ws/src/robot_navigation/sql/createRawData.sql
    
```
![result](./img/raw_data.png)

### 2.  Start sensor node:
```
    roslaunch robot_navigation office_world.launch
    rosrun rosrun robot_navigation door_status_advertiser
```
![sensor](./img/door_status_advertiser.png)
## 3. Start navigation stack
```
    roslaunch robot_navigation robot_navigation.launch
```
use estimate position tool in rviz to estimate position

## 4. run demo
```
    roslaunch robot_navigation move_demo.launch
```
## work flow
![work flow](./img/scheduler-ros_workflow.png)

## demo

https://www.youtube.com/watch?v=cLfMKVpCcfQ

## rqt graph

![rqt](./img/rosgraph.png)


## TO DO

- use [clock](http://wiki.ros.org/Clock) package to simulate the time

- build global occupation possibility table

- write cost function = Θ1 * occupation possibility+  Θ2 * hour different +  Θ3* distance + Θ4* task priority + Θ5 * battery level + Θ6

## problem

![1](./img/problem1.png)

![2](./img/problem2.png)