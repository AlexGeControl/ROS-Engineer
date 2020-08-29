#include "learning_tf2/static_tf/activity.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

namespace learning_tf2 {

namespace static_tf {

Activity::Activity() 
    : private_nh_("~") {

}

Activity::~Activity() {
}

void Activity::Init() {
    // load params:
    // a. transform frames:
    private_nh_.param(
        "base_laser/base_link", 
        config_.base_link, 
        std::string("base_link")
    );
    private_nh_.param(
        "base_laser/sensor_frame", 
        config_.sensor_frame, 
        std::string("base_laser")
    );
    // b. position:
    double x, y, z;
    private_nh_.param("base_laser/position/x", x, 0.0);
    private_nh_.param("base_laser/position/y", y, 0.0);
    private_nh_.param("base_laser/position/z", z, 0.0);
    config_.position = Eigen::Vector3d(x, y, z);
    // c. orientation:
    double roll, pitch, yaw;
    private_nh_.param("base_laser/orientation/roll", roll, 0.0);
    private_nh_.param("base_laser/orientation/pitch", pitch, 0.0);
    private_nh_.param("base_laser/orientation/yaw", yaw, 0.0);
    config_.orientation = Eigen::Vector3d(roll, pitch, yaw);

    // publish static tf:
    geometry_msgs::TransformStamped static_tf_stamped;

    static_tf_stamped.header.stamp = ros::Time::now();

    static_tf_stamped.header.frame_id = config_.base_link;
    static_tf_stamped.child_frame_id = config_.sensor_frame;

    static_tf_stamped.transform.translation.x = config_.position.x();
    static_tf_stamped.transform.translation.y = config_.position.y();
    static_tf_stamped.transform.translation.z = config_.position.z();

    tf2::Quaternion q;
    q.setRPY(
        config_.orientation(0),
        config_.orientation(1),
        config_.orientation(2)
    );
    static_tf_stamped.transform.rotation.x = q.x();
    static_tf_stamped.transform.rotation.y = q.y();
    static_tf_stamped.transform.rotation.z = q.z();
    static_tf_stamped.transform.rotation.w = q.w();

    static_tf_broadcaster_.sendTransform(static_tf_stamped);
}

} // namespace static_tf

} // namespace learning_tf2