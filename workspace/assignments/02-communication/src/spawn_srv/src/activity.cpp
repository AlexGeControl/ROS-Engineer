#include "activity.hpp"

#include <boost/lexical_cast.hpp>
#include <string>

#include <turtlesim/Spawn.h>

namespace ros_comm {

namespace spawn_srv {

Activity::Activity() 
    : private_nh_("~") {
}

Activity::~Activity() {}

void Activity::Init() {
    // load params:
    private_nh_.param(
        "spawn_turtle/id", 
        config_.id, 
        1
    );

    private_nh_.param(
        "spawn_turtle/pose/x", 
        config_.pose.x, 
        3.0
    );
    private_nh_.param(
        "spawn_turtle/pose/y", 
        config_.pose.y, 
        3.0
    );
    private_nh_.param(
        "spawn_turtle/pose/theta", 
        config_.pose.theta, 
        0.0
    );

    // init client:
    client_ = private_nh_.serviceClient<turtlesim::Spawn>("/spawn");
}

bool Activity::SpawnNewTurtle() {
  turtlesim::Spawn::Request req;
  turtlesim::Spawn::Response res;

  req.x = config_.pose.x;
  req.y = config_.pose.y;
  req.theta = config_.pose.theta;

  const std::string turtle_id = boost::lexical_cast<std::string>(config_.id);
  req.name = "turtle" + turtle_id;

  ros::service::waitForService("/spawn", ros::Duration(5));
  bool status = client_.call(req, res);
  
  return status;
}

} // namespace spawn_srv

} // namespace ros_comm