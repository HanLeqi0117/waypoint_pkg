import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Launch引数宣言
    txt_file_arg = DeclareLaunchArgument("way_txt_file", default_value="/home/ubuntu/ubuntu/waypoint.txt")
    point_distance_arg = DeclareLaunchArgument("point_distance", default_value=4.0)
    deg_thresh_arg = DeclareLaunchArgument("deg_thresh", default_value=15.0)
    deg_chord_arg = DeclareLaunchArgument("deg_chord", default_value=1.0)
    
    # Node宣言
    twist2txt_node = Node(
        package="waypoint_pkg",
        executable="twist2txt",
        name="wayp_twist2txt",
        parameters=[{
            "way_txt_file": LaunchConfiguration("way_txt_file"),
            "point_distance": LaunchConfiguration("point_distance"),
            "deg_thresh": LaunchConfiguration("deg_thresh"),
            "deg_chord": LaunchConfiguration("deg_chord")
        }],
        output="screen"
    )

    return LaunchDescription([
        txt_file_arg,
        point_distance_arg,
        deg_thresh_arg,
        deg_chord_arg,
        twist2txt_node
    ])
