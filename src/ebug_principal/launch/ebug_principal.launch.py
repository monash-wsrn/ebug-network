import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ebug_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # This node is the central controller (i.e., Boids Algorithm)
        Node(
            package='ebug_principal',
            executable='BoidsService',
            name='BoidsService',
        ),


        # TODO add remote nodes launch (i.e., static_transform_publisher instances)
        # View /legacy/ebugnet_ws/src/ebug/ros2_localization/src/localization/launch/remote_nodes.launch.py
    ])