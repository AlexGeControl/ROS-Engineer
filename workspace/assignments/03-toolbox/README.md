# ROS Engineering Theory and Practice: Toolbox -- ROS理论与实践: Toolbox

This is the solution of Assignment 03 of ROS Engineering Theory and Practice [深蓝学院](https://www.shenlanxueyuan.com/course/246).

深蓝学院ROS理论与实践第03讲Toolbox作业解答. 版权归深蓝学院所有. 请勿抄袭.

---

## Solutions

---

### 1. ROS Launch

创建一个learning_launch功能包, 在其中新建launch文件, 使用launch文件, 完成第02讲[ROS Communication](https://github.com/AlexGeControl/ROS-Engineer/tree/master/workspace/assignments/02-communication)三个项目的启动

#### Solution

The solution is available at [here](src/learning_communication). Follow the instructions below to reproduce the results

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

---

### 2. Gazebo

下载Gazebo离线模型库并放置在指定位置. 成功运行Gazebo后, 在系统中添加模型, 测试环境

#### Solution

Gazebo模型下载已集成至Docker环境

```Dockerfile
# download gazebo modes:
RUN mkdir -p /root/.gazebo/models &&
    git clone https://github.com/osrf/gazebo_models.git /root/.gazebo/models
```

使用Repo根目录的Docker环境, 启动Gazebo:

<img src="doc/gazebo/launch-gazebo.png" alt="Launch Gazebo in Workspace" width="%100">

在Gazebo中添加模型的效果如下:

<img src="doc/gazebo/add-model.png" alt="Add Model in Gazebo" width="%100">

---

### 3. Learning TF2

#### Solution

The solution is available at package [turtlesim_control](src/turtlesim_control)
