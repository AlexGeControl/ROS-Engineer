# ROS Engineering Theory and Practice: Robot Description -- ROS理论与实践: Robot Description

This is the solution of Assignment 04 of ROS Engineering Theory and Practice [深蓝学院](https://www.shenlanxueyuan.com/course/246).

深蓝学院ROS理论与实践第04讲Robot Description作业解答. 版权归深蓝学院所有. 请勿抄袭.

---

## Solutions

---

### 1. Create a robot model for differential drive mobile robot

#### Solution

The solution is available at [here](src/robot_description). Follow the instructions below to reproduce the results

```bash
# build release:
catkin config --install && catkin build learning_communication
# set up session:
source install/setup.bash
# launch:
# a. pub-sub:
roslaunch learning_communication pub_sub.launch
# b. spawn service: 
roslaunch learning_communication spawn_srv.launch
# c. turtlesim control service:
roslaunch learning_communication turtlesim_control.launch
```
