# ROS Engineering Theory and Practice: Communication -- ROS理论与实践: Communication

This is the solution of Assignment 02 of ROS Engineering Theory and Practice [深蓝学院](https://www.shenlanxueyuan.com/course/246).

深蓝学院ROS理论与实践第02讲Communication作业解答. 版权归深蓝学院所有. 请勿抄袭.

---

## Solutions

---

### 1. Pub & Sub

创建一个节点, 实现一个订阅者和一个发布者，完成以下功能

* 发布者--发布海龟速度指令, 控制海龟进行圆周运动
* 订阅者--订阅海龟的位置信息, 并在终端中周期打印输出

#### Solution

The solution is available at package [pub_sub](src/pub_sub)

To change `turtle pose output frequency` and `turtle motion parameters`, change the following fields in the [config file](src/pub_sub/config/pub_sub.yaml)

```yaml
turtle:
    id: 1
    pose:
        downsample_rate: 10
    motion:
        # circular motion frequency in Hz:
        frequency: 0.1
        # circular motion radius in meters:
        radius: 0.5
```

Use the following bash commands to build and launch the solution

```bash
catkin build pub_sub && roslaunch pub_sub pub_sub.launch
```

---

### 2. Service

创建一个节点, 实现一个客户端, 完成以下功能:

* 客户端--请求海龟诞生服务, 在仿真器中产生一只新的海龟

#### Solution

The solution is available at package [spawn_srv](src/spawn_srv)

To change `new turtle ID` and `turtle initial pose`, change the following fields in the [config file](src/spawn_srv/config/spawn_srv.yaml)

```yaml
spawn_turtle:
    id: 2
    pose:
        x: 8.0
        y: 8.0
        theta: 0.0 
```

Use the following bash commands to build and launch the solution

```bash
catkin build spawn_srv && roslaunch spawn_srv spawn_srv.launch
```

---

### 3. TurtleSim API Gateway

实现一个海龟运动控制的功能包, 功能包应具备以下功能:

* 海龟生成--通过命令行发送新生海龟的名字，即可在仿真器中产生一只新海龟, 且新海龟的位姿与已有海龟位姿并不重叠
* 运动控制--通过命令行发送指令，控制仿真器中任意海龟圆周运动的启动/停止, 海龟运动速度可通过命令行控制 