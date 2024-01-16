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
    PointStamped, PoseStamped, Pose
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

class WayopintMode():
    NORMAL = 0
    SEARCH = 1
    CANCEL = 2
    DIRECT = 3
    STOP = 4
    SIGNAL = 5

def vector3_to_point(vector3 : Vector3) -> Point:
    point = Point()
    point.x = vector3.x
    point.y = vector3.y
    point.z = vector3.z
    
    return point

def pose_to_waypoint(pose : Pose, vector : Vector3 = None) -> dict:
    waypoint = {}
    rpy = euler_from_quaternion(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,        
    )
    if vector is not None:
        waypoint['pos_x'] = vector.x
        waypoint['pos_y'] = vector.y
        waypoint['pos_z'] = vector.z
    else:
        waypoint['pos_x'] = pose.position.x
        waypoint['pos_y'] = pose.position.y
        waypoint['pos_z'] = pose.position.z
        
    waypoint['quat_x'] = pose.orientation.x
    waypoint['quat_y'] = pose.orientation.y
    waypoint['quat_z'] = pose.orientation.z
    waypoint['quat_w'] = pose.orientation.w
    waypoint['roll'] = rpy[0]
    waypoint['pitch'] = rpy[1]
    waypoint['yaw'] = rpy[2]
    waypoint['mode'] = WayopintMode.NORMAL
    
    return waypoint
