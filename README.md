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


### 2. Set up

```
    roslaunch turtlebot3_gazebo multi_turtlebot3.launch
```

Start navigation stack

```
    roslaunch turtlebot3_gazebo multi.launch

```

Optional: Use keyboard to controll one robot 

```
rostopic pub /tb3_0"}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'

```

## 3. run demo
Run demo

```
    roslaunch robot_navigation move_demo.launch
```

Generate a execute task

1. Use task geterator node

```
    roslaunch robot_navigation generate_execute_task.launch
```

2. Use sql script

## ALL figure

in draw.io

https://1drv.ms/u/s!AvKeK97aLoV37WDBLo7S3wfbDctK


## Relative work

https://ieeexplore.ieee.org/abstract/document/7992870

When the time of the server reaches the request time, the server performs the task allocation. 

https://www.cs.utexas.edu/~pstone/Papers/bib2html-links/AAMAS17-Zhang.pdf

https://ieeexplore.ieee.org/abstract/document/8461113


## battery module


## Task type

<!-- ![TaskTypes](./img/robot-TaskTypes.png) -->
