#include <ros/ros.h>

#include "learning_tf2/sensor/activity.hpp"

int main(int argc, char** argv) {
    std::string node_name{"sensor_node"};
    ros::init(argc, argv, node_name);
    
    learning_tf2::sensor::Activity activity;

    activity.Init();
    
    // publish T_world_base_link at 10 Hz:
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        activity.PublishPointCloud();
        loop_rate.sleep();
    } 
    
    return EXIT_SUCCESS;
}