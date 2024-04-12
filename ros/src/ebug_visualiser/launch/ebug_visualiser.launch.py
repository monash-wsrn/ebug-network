import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # PyGame Display visualisation of arena state
    PyGameVisualisation = Node(
        package = 'ebug_visualiser',
        executable = 'PyGameDisplay',
        name = 'PyGameDisplay',
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ebug_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        PyGameVisualisation
    ])