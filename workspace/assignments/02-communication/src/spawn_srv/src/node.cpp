#include "activity.hpp"
#include <ros/ros.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv) {
    std::string node_name{"spawn_srv_node"};
    ros::init(argc, argv, node_name);
    
    ros_comm::spawn_srv::Activity activity;

    activity.Init();
    bool success = activity.SpawnNewTurtle();

    if (!success) {
        ROS_ERROR("Could not spawn new turtle.");
    }

    return EXIT_SUCCESS;
}