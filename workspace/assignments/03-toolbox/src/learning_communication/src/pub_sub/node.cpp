#include "pub_sub/activity.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    std::string node_name{"pub_sub_node"};
    ros::init(argc, argv, node_name);
    
    ros_comm::pub_sub::Activity activity;

    activity.Init();
    
    // 10 Hz:
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        // publish cmd_vel for target turtle:
        activity.PublishCmdVel();
        loop_rate.sleep();
    } 
    
    return EXIT_SUCCESS;
}