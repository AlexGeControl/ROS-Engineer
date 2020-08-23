#include "activity.hpp"
#include <geometry_msgs/Twist.h>

namespace ros_comm {

namespace pub_sub {

Activity::Activity()
    : private_nh_("~") {
}

Activity::~Activity() {}

void Activity::Init() {
    // load params:
    // a. turtle ID:
    private_nh_.param(
        "turtle/id", 
        config_.id, 
        1
    );
    // b. pose downsample rate:
    private_nh_.param(
        "turtle/pose/downsample_rate", 
        config_.pose.downsample_rate, 
        10
    );
    // c. motion config:
    double frequency, radius;
    private_nh_.param(
        "turtle/motion/frequency", 
        frequency, 
        0.5
    );
    private_nh_.param(
        "turtle/motion/radius", 
        radius, 
        0.1
    );

    // set topic names:
    const std::string turtle_id = boost::lexical_cast<std::string>(config_.id);
    std::string pose_topic_name = "/turtle" + turtle_id + "/pose";
    std::string cmd_vel_topic_name = "/turtle" + turtle_id + "/cmd_vel";

    // set up motion:
    config_.motion.w = 2* M_PI * frequency;
    config_.motion.v = config_.motion.w*radius;
    
    // only keep the latest:
    sub_ = private_nh_.subscribe(
        pose_topic_name, 1, &Activity::TurtlePoseCB, this
    );
    pub_ = private_nh_.advertise<geometry_msgs::Twist>(
        cmd_vel_topic_name, 25
    );
};

void Activity::PublishCmdVel(void) {
    geometry_msgs::Twist msg;

    msg.linear.x = config_.motion.v;
    msg.angular.z = config_.motion.w;

    pub_.publish(msg);
}

void Activity::TurtlePoseCB(const turtlesim::Pose::ConstPtr& msg) {
    static size_t msg_count = 0;

    ++msg_count;
    if (msg_count == config_.pose.downsample_rate) {
        ROS_WARN(
            "Latest turtle %d pose\n"
            "\tPose, x: %.3f\n"
            "\tPose, y: %.3f\n"
            "\tPose, theta: %.3f\n"
            "\tVelocity, linear: %.3f\n"
            "\tVelocity, angular: %.3f\n",
            config_.id,
            msg->x, msg->y, msg->theta,
            msg->linear_velocity, msg->angular_velocity
        );

        msg_count = 0;
    }
};

} // namespace pub_sub

} // namespace ros_comm