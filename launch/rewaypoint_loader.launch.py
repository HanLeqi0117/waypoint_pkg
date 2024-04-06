import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch引数宣言
    reway_txt_file_arg = DeclareLaunchArgument("reway_txt_file", default_value="/home/ubuntu/ubuntu/rewaypoint.txt")
    remap_file_path_arg = DeclareLaunchArgument("remap_file_path", default_value="/home/ubuntu/ubuntu/remap/remap.yaml")
    
    # Node宣言
    txt2marker_node = Node(
        package="waypoint_pkg",
        executable="txt2marker",
        name="reway_auto",
        parameters=[{
            "reway_txt_file": LaunchConfiguration("reway_txt_file"),
        }],
        output="screen"
    )
    map_server_node=Node(
        package="nav2_map_server",
        executable="map_server",
        name="wayp_marker_server",
        parameters=[{
            "remap_file_path": LaunchConfiguration("remap_file_path")
        }],
        output="screen"
    )    
 
    return LaunchDescription([
        reway_txt_file_arg,
        remap_file_path_arg,
        txt2marker_node,
        map_server_node
    ])
