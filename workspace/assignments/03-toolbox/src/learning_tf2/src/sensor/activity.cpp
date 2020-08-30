#include "learning_tf2/sensor/activity.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

namespace learning_tf2 {

namespace sensor {

Activity::Activity() 
    : private_nh_("~"), tf_listener_(tf_buffer_)  {

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
    private_nh_.param(
        "base_link", 
        config_.base_link, 
        std::string("base_link")
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

    try{
        // identify target pose:
        geometry_msgs::TransformStamped tf_stamped = tf_buffer_.lookupTransform(
            config_.base_link, config_.sensor_frame,
            ros::Time(0)
        );
        
        tf2::Stamped<tf2::Transform> transform;
        tf2::fromMsg(tf_stamped, transform);
        
        for (size_t i = 0; i < raw_scan.points.size (); ++i) {         
            raw_scan.points[i].x = 0.1 * (i + 1);         
            raw_scan.points[i].y = 0.0;         
            raw_scan.points[i].z = 0.0;

            tf2::Vector3 raw_scan_point(
                raw_scan.points[i].x,
                raw_scan.points[i].y,
                raw_scan.points[i].z
            );

            tf2::Vector3 obs_in_base_link = transform * raw_scan_point;

            ROS_WARN(
                "\tObservation %lu in body frame: %.4f, %.4f, %.4f", 
                i + 1,
                obs_in_base_link.getX(), obs_in_base_link.getY(), obs_in_base_link.getZ() 
            );
        } 

        // convert to ros message:
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(raw_scan, msg);
        msg.header.frame_id = config_.sensor_frame;

        pub_.publish(msg);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }
}

} // namespace sensor

} // namespace learning_tf2