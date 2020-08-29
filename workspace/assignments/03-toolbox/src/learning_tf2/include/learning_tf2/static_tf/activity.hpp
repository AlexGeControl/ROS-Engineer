#ifndef LEARNING_TF2_STATIC_TF_ACTIVITY_HPP
#define LEARNING_TF2_STATIC_TF_ACTIVITY_HPP

#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <ros/ros.h>

#include <tf2_ros/static_transform_broadcaster.h>

namespace learning_tf2 {

namespace static_tf {

struct Config {
    std::string base_link;
    std::string sensor_frame;

    Eigen::Vector3d position;
    Eigen::Vector3d orientation;
};

class Activity {
public:
    Activity();
    ~Activity();

    void Init();

private:
    Config config_;
    
    ros::NodeHandle private_nh_;

    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
};

} // namespace static_tf

} // namespace learning_tf2

#endif  // LEARNING_TF2_STATIC_TF_ACTIVITY_HPP