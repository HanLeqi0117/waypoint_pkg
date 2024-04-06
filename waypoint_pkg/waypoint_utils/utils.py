import rclpy, ruamel.yaml, os, numpy

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

class WaypointMode():
    NORMAL = 0
    SEARCH = 1
    CANCEL = 2
    DIRECT = 3
    STOP = 4
    SIGNAL = 5
    
class Waypoint(WaypointMode):
    pos_x : float
    pos_y : float
    pos_z : float
    quat_x : float
    quat_y : float
    quat_z : float
    quat_w : float
    roll : float
    pitch : float
    yaw : float
    longitude : float
    latitude : float
    mode : int
    
    waypoint = {}
    waypoints = {'waypoints' : [waypoint]}

def vector3_to_point(vector3 : Vector3) -> Point:
    point = Point()
    point.x = vector3.x
    point.y = vector3.y
    point.z = vector3.z
    
    return point

def pose_to_waypoint(pose : Pose, vector : Vector3 = None) -> dict:
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
    Creates a geographic_msgs/msg/GeoPose object from latitude, longitude and yaw
    """
    geopose = GeoPose()
    geopose.position.latitude = latitude
    geopose.position.longitude = longitude
    geopose.orientation = quaternion_from_euler(0.0, 0.0, yaw)
    return geopose

def get_midlatlon(waypoint_A : dict, waypoint_B : dict) -> [float, float]:
    
    # 地球の楕円体モデルのパラメータ (WGS 84)
    a = 6378137.0  # 長軸 (m)
    b = 6356752.3142  # 短軸 (m)
    f = (a - b) / a  # 扁平率
    
    lat_A = waypoint_A["latitude"]
    lon_A = waypoint_A["longitude"]
    lat_B = waypoint_B["latitude"]
    lon_B = waypoint_B["longitude"]

    # 経緯度を弧度に変換
    lat_A_rad = math.radians(lat_A)
    lon_A_rad = math.radians(lon_A)
    lat_B_rad = math.radians(lat_B)
    lon_B_rad = math.radians(lon_B)

    # 直交座標に変換
    sin_lat_A = math.sin(lat_A_rad)
    cos_lat_A = math.cos(lat_A_rad)
    sin_lon_A = math.sin(lon_A_rad)
    cos_lon_A = math.cos(lon_A_rad)

    sin_lat_B = math.sin(lat_B_rad)
    cos_lat_B = math.cos(lat_B_rad)
    sin_lon_B = math.sin(lon_B_rad)
    cos_lon_B = math.cos(lon_B_rad)

    x_A = a * cos_lat_A * cos_lon_A
    y_A = a * cos_lat_A * sin_lon_A
    z_A = b * f**2 * sin_lat_A

    x_B = a * cos_lat_B * cos_lon_B
    y_B = a * cos_lat_B * sin_lon_B
    z_B = b * f**2 * sin_lat_B

    # 中点の直交座標を計算
    x_mid = (x_A + x_B) / 2
    y_mid = (y_A + y_B) / 2
    z_mid = (z_A + z_B) / 2

    # 中点の経緯度座標を逆変換
    lat_mid_rad = math.atan2(z_mid, math.sqrt(x_mid**2 + y_mid**2))
    lon_mid_rad = math.atan2(y_mid, x_mid)

    # 弧度を度に変換
    lat_mid = math.degrees(lat_mid_rad)
    lon_mid = math.degrees(lon_mid_rad)

    return [lat_mid, lon_mid]

def latlon_while_waypoint_changed(waypoint_A : dict, waypoint_B : dict) -> [float, float]:
    # 地球の楕円体モデルのパラメータ (WGS 84)
    a = 6378137.0  # 長軸 (m)
    b = 6356752.3142  # 短軸 (m)
    f = (a - b) / a  # 扁平率
    
    lat_start = waypoint_A["latitude"]
    lon_start = waypoint_A["longitude"]
    
    vector = numpy.array([waypoint_B["pos_x"], waypoint_B["pos_y"]]) - numpy.array([waypoint_A["pos_x"], waypoint_A["pos_y"]])
    rotation_angle = math.atan2(vector[1], vector[0])
    distance = math.hypot(
        waypoint_A["pos_x"] - waypoint_B["pos_x"],
        waypoint_A["pos_y"] - waypoint_B["pos_y"]
    )

    # 経緯度を弧度に変換
    lat_start_rad = math.radians(lat_start)
    lon_start_rad = math.radians(lon_start)

    # 距離をメートルから弧度に変換
    distance_rad = distance / a

    # 中点からの新しい点の直交座標を計算
    lat_dest_rad = math.asin(math.sin(lat_start_rad) * math.cos(distance_rad) +
                             math.cos(lat_start_rad) * math.sin(distance_rad) * math.cos(rotation_angle))
    
    lon_dest_rad = lon_start_rad + math.atan2(math.sin(rotation_angle) * math.sin(distance_rad) * math.cos(lat_start_rad),
                                              math.cos(distance_rad) - math.sin(lat_start_rad) * math.sin(lat_dest_rad))
    
    # 楕円体の短軸方向の変化を考慮
    N = a / math.sqrt(1 - f * (2 - f) * math.sin(lat_start_rad)**2)
    
    x_dest = (N + distance) * math.cos(lat_dest_rad) * math.cos(lon_dest_rad)
    y_dest = (N + distance) * math.cos(lat_dest_rad) * math.sin(lon_dest_rad)
    z_dest = (N * (1 - f)**2 + distance) * math.sin(lat_dest_rad)
    
    # 弧度を度に変換
    lat_dest = math.degrees(lat_dest_rad)
    lon_dest = math.degrees(lon_dest_rad)

    return [lat_dest, lon_dest]

def get_dist_between_geos(lat1 : float, lon1 : float, lat2 : float, lon2 : float):
    # 緯度経度をラジアンに変換
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    # 地球の半径（単位: メートル）
    radius = 6371008.8  # 6,371 km

    # 距離計算
    distance = radius * c

    return distance