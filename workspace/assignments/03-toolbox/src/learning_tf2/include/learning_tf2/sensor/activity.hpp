#ifndef LEARNING_TF2_SENSOR_ACTIVITY_HPP
#define LEARNING_TF2_SENSOR_ACTIVITY_HPP

#include <string>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

namespace learning_tf2 {

namespace sensor {

struct Config {
    std::string topic_name;
    std::string sensor_frame;
    std::string base_link;
};

class Activity {
public:
    Activity();
    ~Activity();

    void Init();
    void PublishPointCloud(void);

private:
    Config config_;
    
    ros::NodeHandle private_nh_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Publisher pub_;
};

} // namespace sensor

} // namespace learning_tf2

#endif  // LEARNING_TF2_SENSOR_ACTIVITY_HPP