#include "turtlesim_control/activity.hpp"

#include <ros/ros.h>

#include <boost/lexical_cast.hpp>
#include <string>

int main(int argc, char** argv) {
    std::string node_name{"turtlesim_control_node"};
    ros::init(argc, argv, node_name);
    
    ros_comm::turtlesim_control::Activity activity;

    activity.Init();

    // cmd_vel rate at 25 Hz:
    ros::Rate loop_rate(25);
    while (ros::ok())
    {
        ros::spinOnce();
        // publish cmd_vel for existing turtles:
        activity.PublishCmdVels();
        loop_rate.sleep();
    } 
    
    return EXIT_SUCCESS;
}