import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # PyGame Display visualisation of arena state
    PyGameVisualisation = Node(
        package = 'ebug',
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