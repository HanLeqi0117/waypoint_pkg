#ifndef WAYPOINT_PKG_UTLILIS_HPP_
#define WAYPOINT_PKG_UTLILIS_HPP_

// Cライブラリ
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>

// ROS2ライブラリ
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Vector3.h>
#include <waypoint_pkg_interfaces/msg/waypoint.hpp>
#include <waypoint_pkg_interfaces/srv/waypoints_update.hpp>

using namespace geometry_msgs::msg;
using namespace visualization_msgs::msg;
using namespace waypoint_pkg_interfaces::msg;
using namespace waypoint_pkg_interfaces::srv;
using namespace std::chrono_literals;
using namespace std::placeholders;

#endif