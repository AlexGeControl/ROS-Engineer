#include <ros/ros.h>

#include "learning_tf2/static_tf/activity.hpp"

int main(int argc, char** argv) {
    std::string node_name{"static_tf_node"};
    ros::init(argc, argv, node_name);
    
    learning_tf2::static_tf::Activity activity;

    activity.Init();
    
    // that's it:
    ros::spin();
    
    return EXIT_SUCCESS;
}