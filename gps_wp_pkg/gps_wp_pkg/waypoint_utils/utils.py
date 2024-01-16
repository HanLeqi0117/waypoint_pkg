import rclpy, ruamel.yaml, os

from rclpy.node import Node
from rclpy.duration import Duration
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.time import Time
from rclpy.qos_overriding_options import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import (
    Point, PoseWithCovariance, Vector3,
    PoseWithCovarianceStamped, Quaternion,
    PointStamped, PoseStamped
)
from sensor_msgs.msg import NavSatFix, Imu
from tf2_geometry_msgs import tf2_geometry_msgs
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarkerControl
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer, TransformStamped
from waypoint_pkg_interfaces.msg import Waypoint
from interactive_markers.menu_handler import *
from interactive_markers.interactive_marker_server import *
from tf_transformations import euler_from_quaternion
import math

def get_euler(quat = Quaternion()) -> tuple:
    return euler_from_quaternion([
        quat.x, quat.y, quat.z, quat.w
    ])

def vector3_to_point(vector3 = Vector3()) -> Point:
    point = Point()
    point.x = vector3.x
    point.y = vector3.y
    point.z = vector3.z
    
    return point

def pose_to_waypoint(pose = PoseStamped()) -> dict:
    waypoint = {}
    rpy = get_euler(pose.pose.orientation)
    waypoint['pos_x'] = pose.pose.position.x
    waypoint['pos_y'] = pose.pose.position.y
    waypoint['pos_z'] = pose.pose.position.z
    waypoint['quat_x'] = pose.pose.orientation.x
    waypoint['quat_y'] = pose.pose.orientation.y
    waypoint['quat_z'] = pose.pose.orientation.z
    waypoint['quat_w'] = pose.pose.orientation.w
    waypoint['roll'] = rpy[0]
    waypoint['pitch'] = rpy[1]
    waypoint['yaw'] = rpy[2]
    waypoint['mode'] = 0
    
    return waypoint
