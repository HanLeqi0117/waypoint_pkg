import rclpy, ruamel.yaml, os, numpy, pycurl
from geographiclib.geodesic import Geodesic

from rclpy.node import Node
from rclpy.duration import Duration
from rcl_interfaces.msg import ParameterDescriptor
import rclpy.node
from rclpy.time import Time
from rclpy.qos_overriding_options import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import (
    Point, PoseWithCovariance, Vector3,
    PoseWithCovarianceStamped, Quaternion,
    PointStamped, PoseStamped, Pose,
    PolygonStamped
)
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPose
from sensor_msgs.msg import NavSatFix, Imu
from tf2_geometry_msgs import tf2_geometry_msgs
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarkerControl
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer, TransformStamped
from waypoint_pkg_interfaces.msg import Waypoint
from interactive_markers.menu_handler import *
from interactive_markers.interactive_marker_server import *
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math


WGS84 = {"a" : 6378137.0, "b" : 6356752.314245, "f" : 1 / 298.257223563}
'''
the parameter of WGS84
'''

class WaypointMode():
    """
        Describe the Mode fo Waypoint with enumeration
        
        Attrs:
        - NORMAL: Noraml navigation mode without specified task
        - SEARCH: Search some objects using image recognition
        - CANCEL: Unkown
        - DIRECT: Unkown
        - STOP: Stop moving when robot has arrived to this waypoint
        - SIGNAL: Execute the task which is the recognition of traffic light
    """    
    
    NORMAL = 0
    SEARCH = 1
    CANCEL = 2
    DIRECT = 3
    STOP = 4
    SIGNAL = 5
    
class Waypoint(WaypointMode):
    """
        Description of Waypoint
        
        Attrs:
        - waypoint: a dictionary is like (key : value):
            - pos_x : position_x (float)
            - pos_y : position_y (float)
            - pos_z : position_z (float)
            - quat_x : quaternion_x (float)
            - quat_y : quaternion_y (float)
            - quat_z : quaternion_z (float)
            - quat_w : quaternion_w (float)
            - roll : roll (float)
            - pitch : pitch (float)
            - yaw : yaw (float)
            - longitude : longitude (float)
            - latitude : latitude (float)
            - mode : mode (int)
        - wayponits : a dictionary is like:
            - dict["waypoins" : [{}]]
    """
    
    waypoint = dict[str, int|float]
    waypoints = dict[str, list[dict[str, int|float]]]
    
    def get_initial_waypoint():
        mode = int(Waypoint.NORMAL)
        waypoint = {
            "pos_x" : 0.0,
            "pos_y" : 0.0,
            "pos_z" : 0.0,
            "quat_x" : 0.0,
            "quat_y" : 0.0,
            "quat_z" : 0.0,
            "quat_w" : 0.0,
            "roll" : 0.0,
            "pitch" : 0.0,
            "yaw" : 0.0,
            "longitude" : 0.0,
            "latitude" : 0.0,
            "mode" : mode
        }
        return waypoint
    
    def get_initial_waypoints(key_name : str):
        waypoints = {key_name : []}
        return waypoints
    
        
def vector3_to_point(vector3 : Vector3) -> Point:
    """
        Convert Vector3 to Point. Both Vector3 and Point is a kind of ROS Message Interface
        
        Args:
        - vector3: geometry_msgs/msg/Vector3

        Return:
        - point: geometry_msgs/msg/Point
    """
    point = Point()
    point.x = vector3.x
    point.y = vector3.y
    point.z = vector3.z
    
    return point

def pose_to_waypoint(pose : Pose, vector : Vector3 = None) -> dict:
    """
        According to geometry_msgs/msg/Pose and Vector3(optional), generate a waypoint whose message type is dictionary
        
        Args:
        - pose: geometry_msgs/msg/Pose
        - vector: geometry_msgs/msg/Vector3(optional). if specified, the orientation settings will follow it accordingly. 

        Return:
        - waypoint: dictionary with localization of robot such as point, orientation and geopose
    """
    waypoint = {}
    rpy = euler_from_quaternion([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,        
    ])
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
    waypoint['mode'] = WaypointMode.NORMAL
    
    return waypoint

def latLonYaw2Geopose(latitude: float, longitude: float, yaw: float = 0.0) -> GeoPose:
    """
        Return a GeoPose Message using latitude, longitude and yaw
        
        Args:
        - latitude: float
        - longitude: float
        - yaw: 0.0 by default

        Return:
        - geopose: geographic_msgs/msg/GeoPose
    """
    geopose = GeoPose()
    geopose.position.latitude = latitude
    geopose.position.longitude = longitude
    geopose.orientation = quaternion_from_euler(0.0, 0.0, yaw)
    return geopose

def get_midlatlon(lat1 : float, lon1 : float, lat2 : float, lon2 : float) -> list[float, float]:
    """
        Get the latitude and longitude at the middle position between two waypoints
        
        Args:
        - lat1: latitude of Geopoint No. 1
        - lon1: longitude of Geopoint No. 1
        - lat2: latitude of Geopoint No. 2
        - lon2: longitude of Geopoint No. 2

        Return:
        - [latitude, longitude] of Geopoint No. 3 which is at the middle position between No. 1 and No. 2
    """
    
    # Karney's inverse formula
    geod = Geodesic(WGS84["a"], WGS84["f"])
    g = geod.Inverse(lat1, lon1, lat2, lon2)
    s12 = g["s12"]
    azi1 = g["azi1"]
    
    result = geod.Line(lat1, lon1, azi1).Position(0.5 * s12)

    return [result['lat2'], result['lon2']]

def latlon_while_waypoint_altered(waypoint_A : dict, waypoint_B : dict) -> list[float, float]:
    """
        When a waypoint is altered, the latitude and longitude of this waypoint will be with the change of pose
        
        Args:
        - waypoint_A: waypoint before altered
        - waypoint_B: waypoint altered

        Return:
        - [latitude, longitude] which the waypoint altered should be with
    """    
    
    lat1 = waypoint_A["latitude"]
    lon1 = waypoint_A["longitude"]
    
    vector = numpy.array([waypoint_B["pos_x"], waypoint_B["pos_y"]]) - numpy.array([waypoint_A["pos_x"], waypoint_A["pos_y"]])
    azi1 = math.atan2(vector[1], vector[0])
    s12 = math.hypot(
        waypoint_A["pos_x"] - waypoint_B["pos_x"],
        waypoint_A["pos_y"] - waypoint_B["pos_y"]
    )
    
    geod = Geodesic(WGS84["a"], WGS84["f"])
    result = geod.Line(lat1, lon1, azi1).Position(s12)

    return [result["lat2"], result["lon2"]]

def get_dist_between_geos(lat1 : float, lon1 : float, lat2 : float, lon2 : float) -> float:
    """
        Get the distance between two geopoints, unit: m
        
        Args:
        - lat1: latitude of Geopoint No. 1
        - lon1: longitude of Geopoint No. 1
        - lat2: latitude of Geopoint No. 2
        - lon2: longitude of Geopoint No. 2

        Return:
        - Distance between two geopoints [m]
    """     
    
    geod = Geodesic(WGS84["a"], WGS84["f"])
    result = geod.Inverse(lat1, lon1, lat2, lon2)

    return result["s12"]

def get_yaw_with_geos(lat1 : float, lon1 : float, lat2 : float, lon2 : float) -> list:
    geod = Geodesic(WGS84["a"], WGS84["f"])
    result = geod.Inverse(lat1, lon1, lat2, lon2)
    
    return result["azi1"]


def get_geolocation(rclpy_node_name : str, place : str) -> list[float] | None:
    """
        Get Geolocation from Internet by giving a place name
        
        Args:
        - rclpy_node : A rclpy.node.Node Object for ros_logger
        - place : Place name which will give a Geolocation back
            
        Return: A list with Geolocation
        - [Latitude, Longitude]
    """   
    
    c = pycurl.Curl()
    
    if " " in place:
        place = place.split(" ")
        place = "%20".join(place)
        
    c.setopt(pycurl.URL, "https://www.geocoding.jp/kml/?q={}".format(place))
    
    if os.environ["http_proxy"] is not None:
        c.setopt(pycurl.PROXY, os.environ["http_proxy"])
        
    s = c.perform_rs()
    
    if s == None:
        rclpy.node.get_logger(rclpy_node_name).info("Couldn't connect to URL, Please check the Network")
        return None
    
    lines = s.split("\n")
    for line in lines:
        if "coordinates" in line:
            line = line[13:]
            line = line[:-14]
            longitude, latitude = \
                float(line.split(",")[0]), \
                float(line.split(",")[1])
            
            return [latitude, longitude]