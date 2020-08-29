#ifndef ROS_COMM_TURTLESIM_CONTROL_ACTIVITY_H
#define ROS_COMM_TURTLESIM_CONTROL_ACTIVITY_H

#include <string>
#include <unordered_map>

#include <ros/ros.h>

#include "learning_communication/SpawnTurtle.h"
#include "learning_communication/SetSpeed.h"

namespace ros_comm {

namespace turtlesim_control {

struct Config {
    struct {
        size_t N;
        double grid_size;
        size_t num_turtles;
        size_t max_turtles;
    } spawn;
};

struct State {
    std::string name;

    ros::Publisher pub;

    struct {
        double w;
        double v;
    } velocity;
};

class Activity {
public:
    Activity();
    ~Activity();

    void Init(void);
    bool SpawnTurtle(const std::string &name);
    void PublishCmdVels(void);

    size_t GetMaxNumTurtles(void) const { return config_.spawn.max_turtles; }
    size_t GetCurrNumTurtles(void) const { return config_.spawn.num_turtles; }
private:
    Config config_;

    void ResetWorld(void);

    bool SpawnTurtle(
        learning_communication::SpawnTurtle::Request &spawn_req,
        learning_communication::SpawnTurtle::Response &spawn_res
    );

    bool SetSpeed(
        learning_communication::SetSpeed::Request &set_speed_req,
        learning_communication::SetSpeed::Response &set_speed_res
    );

    ros::NodeHandle private_nh_;

    std::unordered_map<size_t, State> turtle_state;

    struct {
        ros::ServiceServer spawn_;
        ros::ServiceServer motion_control_;
    } service;

    ros::ServiceClient client_;
};

} // namespace turtlesim_control

} // namespace ros_comm

#endif  // ROS_COMM_TURTLESIM_CONTROL_ACTIVITY_H