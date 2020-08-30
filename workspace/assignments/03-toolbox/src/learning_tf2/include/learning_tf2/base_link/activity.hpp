#ifndef LEARNING_TF2_BASE_LINK_ACTIVITY_HPP
#define LEARNING_TF2_BASE_LINK_ACTIVITY_HPP

#include <string>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <turtlesim/Pose.h>

namespace learning_tf2 {

namespace base_link {

struct Config {
    std::string source_name;
    std::string target_name;

    std::string map_frame;
    std::string base_link;
    std::string target_frame;
};

class Activity {
public:
    Activity();
    ~Activity();

    void Init(void);
    
private:
    void SourcePoseCB(const turtlesim::PoseConstPtr &msg);
    void TargetPoseCB(const turtlesim::PoseConstPtr &msg);

    Config config_;
    
    ros::NodeHandle private_nh_;

    ros::Subscriber sub_source_, sub_target_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
};

} // namespace base_link

} // namespace learning_tf2

#endif  // LEARNING_TF2_BASE_LINK_ACTIVITY_HPP