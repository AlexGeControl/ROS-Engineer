#ifndef ROS_COMM_PUB_SUB_ACTIVITY_H
#define ROS_COMM_PUB_SUB_ACTIVITY_H

#include <string>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <turtlesim/Pose.h>

namespace ros_comm {

namespace pub_sub {

struct Config {
    int id;

    struct {
        int downsample_rate;
    } pose;

    struct {
        double v;
        double w;
    } motion;
};

class Activity {
public:
    Activity();
    ~Activity();

    void Init();
    void PublishCmdVel();

private:
    void TurtlePoseCB(const turtlesim::Pose::ConstPtr& msg);

    Config config_;

    ros::NodeHandle private_nh_;

    ros::Subscriber sub_;
    ros::Publisher pub_;
};

} // namespace pub_sub

} // namespace ros_comm

#endif  // ROS_COMM_PUB_SUB_ACTIVITY_H