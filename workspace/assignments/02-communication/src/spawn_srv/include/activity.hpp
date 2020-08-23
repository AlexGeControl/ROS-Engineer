#ifndef ROS_COMM_SPAWN_SRV_ACTIVITY_H
#define ROS_COMM_SPAWN_SRV_ACTIVITY_H

#include <ros/ros.h>

namespace ros_comm {

namespace spawn_srv {

struct Config {
    int id;

    struct {
        double x;
        double y;
        double theta;
    } pose;
};

class Activity {
public:
    Activity();
    ~Activity();

    void Init();
    bool SpawnNewTurtle();

private:
    Config config_;

    ros::NodeHandle private_nh_;
    ros::ServiceClient client_;
};

} // namespace spawn_srv

} // namespace ros_comm

#endif  // ROS_COMM_SPAWN_SRV_ACTIVITY_H