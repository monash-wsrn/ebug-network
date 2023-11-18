from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable   
import os

def generate_launch_description():
    return LaunchDescription([

        # # launch the ekf node

        
        ### static frames for the apriltags in respect to the centre of beacon (0,0,0) world frame
        # based on https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.11', '--y', '0.0', '--z', '0', '--yaw', '1.5708', '--pitch', '0.0', '--roll', '1.5708', '--frame-id', 'map', '--child-frame-id', 'apriltag_0']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.0', '--y', '0.15', '--z', '0', '--yaw', '3.1416', '--pitch', '0', '--roll', '1.5708', '--frame-id', 'map', '--child-frame-id', 'apriltag_1']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '-0.11', '--y', '0.0', '--z', '0', '--yaw', '-1.5708', '--pitch', '0', '--roll', '1.5708', '--frame-id', 'map', '--child-frame-id', 'apriltag_2']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.0', '--y', '-0.15', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '1.5708', '--frame-id', 'map', '--child-frame-id', 'apriltag_3']
        ),
  
    ])