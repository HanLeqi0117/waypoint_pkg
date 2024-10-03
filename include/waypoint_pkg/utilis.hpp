#ifndef WAYPOINT_PKG_UTLILIS_HPP_
#define WAYPOINT_PKG_UTLILIS_HPP_

// Cライブラリ
#include <map>
#include <memory>
#include <any>
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>

// ROS2ライブラリ
// #include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
// #include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace Waypoint{

enum WaypointMode {
    NORMAL = 0,
    SEARCH = 1,
    CANCEL = 2,
    DIRECT = 3,
    STOP = 4,
    SIGNAL = 5
};

struct Waypoint{
    double pos_x;
    double pos_y;
    double pos_z;
    double quat_x;
    double quat_y;
    double quat_z;
    double quat_w;
    double roll;
    double pitch;
    double yaw;
    double longitude;
    double latitude;
    WaypointMode mode;
};

// typedef std::map<std::string, std::any> Waypoint;
typedef std::map<int, Waypoint> Waypoints;

}


#endif