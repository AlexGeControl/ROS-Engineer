#include "learning_tf2/listener/activity.hpp"

#include <turtlesim/Spawn.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

namespace learning_tf2 {

namespace listener {

Activity::Activity() 
    : private_nh_("~"), tf_listener_(tf_buffer_) {

}

Activity::~Activity() {

}

void Activity::Init() {
    // load params:
    private_nh_.param(
        "source_name", 
        config_.source_name, 
        std::string("turtle1")
    );
    private_nh_.param(
        "target_name", 
        config_.target_name, 
        std::string("turtle2")
    );

    private_nh_.param(
        "source_frame", 
        config_.source_frame, 
        std::string("base_link")
    );
    private_nh_.param(
        "target_frame", 
        config_.target_frame, 
        std::string("target")
    );

    private_nh_.param(
        "control/K_v", 
        config_.K_v, 
        0.5
    );
    private_nh_.param(
        "control/K_w", 
        config_.K_w, 
        4.0
    );

    // init cmd_vel publisher:
    const std::string topic_name = "/" + config_.target_name + "/cmd_vel";
    pub_ = private_nh_.advertise<geometry_msgs::Twist>(topic_name, 10);
}

void Activity::ComputeVelocityCmd(void) {
    try{
        // identify target pose:
        geometry_msgs::TransformStamped tf_stamped = tf_buffer_.lookupTransform(
            config_.target_frame, config_.source_frame,
            ros::Time(0)
        );

        // set goal:
        double target_x = tf_stamped.transform.translation.x;
        double target_y = tf_stamped.transform.translation.y;

        // publish control command:
        geometry_msgs::Twist cmd_vel;

        cmd_vel.linear.x = config_.K_v * hypot(target_x, target_y);
        cmd_vel.angular.z = config_.K_w * atan2(target_y, target_x);

        pub_.publish(cmd_vel);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }
}

} // namespace listener

} // namespace learning_tf2