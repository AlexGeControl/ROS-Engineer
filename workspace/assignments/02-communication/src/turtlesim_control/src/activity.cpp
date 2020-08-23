#include "activity.hpp"


#include <string>

// for spawn service:
#include <turtlesim/Kill.h>
#include <turtlesim/Spawn.h>

// for motion control service:
#include <geometry_msgs/Twist.h>

namespace ros_comm {

Activity::Activity()
    : private_nh_("~") {
}

Activity::~Activity() {}

void Activity::ResetWorld(void) {
    // kill existing turtle:
    turtlesim::Kill::Request req;
    turtlesim::Kill::Response res;

    req.name = "turtle1";
    ros::service::waitForService("/kill", ros::Duration(5));

    ros::ServiceClient client = private_nh_.serviceClient<turtlesim::Kill>("/kill");
    bool status = client.call(req, res);
}

void Activity::Init() {
    // load params:
    // a. workspace layout for turtles:
    private_nh_.param(
        "spawn/grid_size", 
        config_.spawn.grid_size, 
        1.0
    );

    config_.spawn.N = static_cast<size_t>(10.0 / config_.spawn.grid_size);

    config_.spawn.num_turtles = 0;
    config_.spawn.max_turtles = config_.spawn.N*config_.spawn.N;

    // init spawn service:
    service.spawn_ = private_nh_.advertiseService("spawn_turtle", &Activity::SpawnTurtle, this);
    service.motion_control_ = private_nh_.advertiseService("set_speed", &Activity::SetSpeed, this);

    // init turtlesim spawn client:
    client_ = private_nh_.serviceClient<turtlesim::Spawn>("/spawn");

    ResetWorld();
}

bool Activity::SpawnTurtle(const std::string &name) {
    // only spawn new turtle when there is still available grid:
    if (config_.spawn.num_turtles >= config_.spawn.max_turtles) {
        return false;
    }

    turtlesim::Spawn::Request req;
    turtlesim::Spawn::Response res;

    // new turtle ID:
    req.name = name;

    // assign to grid:
    size_t grid_index_x, grid_index_y;

    grid_index_x = config_.spawn.num_turtles % config_.spawn.N;
    grid_index_y = config_.spawn.num_turtles / config_.spawn.N;

    req.x = config_.spawn.grid_size*(grid_index_x + 0.5);
    req.y = config_.spawn.grid_size*(grid_index_y + 0.5);
    req.theta = 0.0;

    ros::service::waitForService("/spawn", ros::Duration(1));
    bool status = client_.call(req, res);
    
    if (status) {
        ++config_.spawn.num_turtles;
    }

    return status;
}

 void Activity::PublishCmdVels(void) {
    static geometry_msgs::Twist msg;

    for (const auto &s: turtle_state) {
        const State &state = s.second;

        msg.angular.z = state.velocity.w;
        msg.linear.x = state.velocity.v;
        
        // publish command:
        state.pub.publish(msg);
    }
 }

bool Activity::SpawnTurtle(
    turtlesim_control::SpawnTurtle::Request &spawn_req,
    turtlesim_control::SpawnTurtle::Response &spawn_res
) {
    // only spawn new turtle when there is still available grid:
    if (config_.spawn.num_turtles >= config_.spawn.max_turtles) {
        return false;
    }

    turtlesim::Spawn::Request req;
    turtlesim::Spawn::Response res;

    // new turtle ID:
    req.name = spawn_req.name;

    // assign to grid:
    size_t grid_index_x, grid_index_y;

    grid_index_x = config_.spawn.num_turtles % config_.spawn.N;
    grid_index_y = config_.spawn.num_turtles / config_.spawn.N;

    req.x = config_.spawn.grid_size*(grid_index_x + 0.5);
    req.y = config_.spawn.grid_size*(grid_index_y + 0.5);
    req.theta = 0.0;

    ros::service::waitForService("/spawn", ros::Duration(1));
    bool status = client_.call(req, res);
    
    if (status) {
        size_t turtle_id = config_.spawn.num_turtles;

        State state;

        // set name:
        state.name = spawn_req.name;
        // set up cmd_vel publisher:
        std::string topic_name = "/" + spawn_req.name + "/cmd_vel";
        state.pub = private_nh_.advertise<geometry_msgs::Twist>(
            topic_name, 1
        );

        // register state:
        turtle_state[turtle_id] = state;

        ++config_.spawn.num_turtles;

        spawn_res.id = turtle_id;
    }

    return status;    
}

bool Activity::SetSpeed(
    turtlesim_control::SetSpeed::Request &set_speed_req,
    turtlesim_control::SetSpeed::Response &set_speed_res
) {
    // get turtle id:
    const size_t turtle_id = set_speed_req.id;

    if (turtle_id < config_.spawn.max_turtles) {
        // update state:
        State &state = turtle_state[turtle_id];

        state.velocity.w = 2* M_PI * set_speed_req.frequency;
        state.velocity.v = state.velocity.w * std::min(
            config_.spawn.grid_size/4, 
            set_speed_req.radius
        );

        set_speed_res.name = state.name;

        return true;
    }

    return false;
}

} // namespace ros_comm