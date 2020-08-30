#include "learning_tf2/base_link/activity.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

namespace learning_tf2 {

namespace base_link {

Activity::Activity() 
    : private_nh_("~") {

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
        "map_frame", 
        config_.map_frame, 
        std::string("world")
    );
    private_nh_.param(
        "base_link", 
        config_.base_link, 
        std::string("base_link")
    );
    private_nh_.param(
        "target_frame", 
        config_.target_frame, 
        std::string("target")
    );

    // init body pose subscriber:
    const std::string source_pose_topic_name = "/" + config_.source_name + "/pose";
    const std::string target_pose_topic_name = "/" + config_.target_name + "/pose";
    sub_source_ = private_nh_.subscribe(source_pose_topic_name, 10, &Activity::SourcePoseCB, this);
    sub_target_ = private_nh_.subscribe(target_pose_topic_name, 10, &Activity::TargetPoseCB, this);
}

void Activity::SourcePoseCB(const turtlesim::PoseConstPtr &msg) {
    geometry_msgs::TransformStamped tf_stamped;

    tf_stamped.header.stamp = ros::Time::now();
    tf_stamped.header.frame_id = config_.map_frame;
    tf_stamped.child_frame_id = config_.base_link;

    tf_stamped.transform.translation.x = msg->x;
    tf_stamped.transform.translation.y = msg->y;
    tf_stamped.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);

    tf_stamped.transform.rotation.x = q.x();
    tf_stamped.transform.rotation.y = q.y();
    tf_stamped.transform.rotation.z = q.z();
    tf_stamped.transform.rotation.w = q.w();

    tf_broadcaster_.sendTransform(tf_stamped);
}

void Activity::TargetPoseCB(const turtlesim::PoseConstPtr &msg) {
    geometry_msgs::TransformStamped tf_stamped;

    tf_stamped.header.stamp = ros::Time::now();
    tf_stamped.header.frame_id = config_.map_frame;
    tf_stamped.child_frame_id = config_.target_frame;

    tf_stamped.transform.translation.x = msg->x;
    tf_stamped.transform.translation.y = msg->y;
    tf_stamped.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);

    tf_stamped.transform.rotation.x = q.x();
    tf_stamped.transform.rotation.y = q.y();
    tf_stamped.transform.rotation.z = q.z();
    tf_stamped.transform.rotation.w = q.w();

    tf_broadcaster_.sendTransform(tf_stamped);
}

} // namespace base_link

} // namespace learning_tf2