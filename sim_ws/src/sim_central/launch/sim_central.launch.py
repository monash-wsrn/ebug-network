import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # This node is the central controller (i.e., Boids Algorithm)
        Node(
            package='sim_central',
            executable='BoidsService',
            name='BoidsService',
        ),

        # This node is the connector between the central controller and an individual robot.
        # Simply duplicate this node and set the robot_id parameter appropriately
        Node(
            package='sim_central',
            executable='RobotController',
            name='RobotController',
            parameters=[
                {"robot_id": '0'},
                {"service_name": 'BoidsService'}
            ]),
    ])