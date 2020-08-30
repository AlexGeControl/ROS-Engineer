#ifndef LEARNING_TF2_LISTENER_ACTIVITY_HPP
#define LEARNING_TF2_LISTENER_ACTIVITY_HPP

#include <string>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>


namespace learning_tf2 {

namespace listener {

struct Config {
    std::string source_name;
    std::string target_name;

    std::string source_frame;
    std::string target_frame;

    double K_v;
    double K_w;
};

class Activity {
public:
    Activity();
    ~Activity();

    void Init();
    void ComputeVelocityCmd(void);

private:
    Config config_;
    
    ros::NodeHandle private_nh_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Publisher pub_;
};

} // namespace listener

} // namespace learning_tf2

#endif  // LEARNING_TF2_LISTENER_ACTIVITY_HPP