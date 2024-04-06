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
    read_file_path_arg = DeclareLaunchArgument("read_file_path", default_value="/home/ubuntu/ubuntu/waypoint.txt")
    write_file_path_arg = DeclareLaunchArgument("write_file_path", default_value="/home/ubuntu/ubuntu/rewaypoint.txt")
    save_service_path_arg = DeclareLaunchArgument("save_service_path", default_value="save_service")
    update_service_name_arg = DeclareLaunchArgument("update_service_name", default_value="update_service")
    remap_file_path_arg = DeclareLaunchArgument("remap_file_path", default_value="/home/ubuntu/ubuntu/remap/remap.yaml")
    
    # Node宣言
    marker_server_node = Node(
        package="waypoint_pkg",
        executable="marker_server",
        name="wayp_marker_server",
        parameters=[{
            "read_file_name": LaunchConfiguration("read_file_name"),
            "write_file_name": LaunchConfiguration("write_file_name"),
            "save_service_name": LaunchConfiguration("save_service_name"),
            "update_service_name": LaunchConfiguration("update_service_name")
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
        read_file_path_arg,
        write_file_path_arg,
        update_service_name_arg,
        save_service_path_arg,
        remap_file_path_arg,
        marker_server_node,
        map_server_node
    ])

