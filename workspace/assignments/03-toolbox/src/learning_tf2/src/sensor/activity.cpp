#include "learning_tf2/sensor/activity.hpp"

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

namespace learning_tf2 {

namespace sensor {

Activity::Activity() 
    : private_nh_("~") {

}

Activity::~Activity() {

}

void Activity::Init() {
    // load params:
    private_nh_.param(
        "topic_name", 
        config_.topic_name, 
        std::string("raw_scan")
    );
    private_nh_.param(
        "sensor_frame", 
        config_.sensor_frame, 
        std::string("base_laser")
    );

    // init raw scan publisher:
    pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>(config_.topic_name, 1);
}

void Activity::PublishPointCloud(void) {
    // create raw scan:
    pcl::PointCloud<pcl::PointXYZ> raw_scan;

    raw_scan.width  = 3;     
    raw_scan.height = 3;     
    raw_scan.points.resize(raw_scan.width * raw_scan.height);     

    for (size_t i = 0; i < raw_scan.points.size (); ++i) {         
        raw_scan.points[i].x = 0.1 * (i + 1);         
        raw_scan.points[i].y = 0.0;         
        raw_scan.points[i].z = 0.0;     
    } 

    // convert to ros message:
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(raw_scan, msg);
    msg.header.frame_id = config_.sensor_frame;

    pub_.publish(msg);
}

} // namespace sensor

} // namespace learning_tf2