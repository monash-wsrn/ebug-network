import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # This node is the central controller (i.e., Boids Algorithm)
    Boids = Node(
        package = 'ebug',
        executable = 'BoidsService',
        name = 'BoidsService',
    )

    Disco = Node(
        package = 'ebug',
        executable = 'DiscoService',
        name = 'DiscoService',
    )

    StaticTransformAprilTag0 = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        arguments = ['--x', '1.08', '--y', '0.7', '--z', '0', '--yaw', '-0.785398', '--pitch', '0.0', '--roll', '1.5708', '--frame-id', 'map', '--child-frame-id', 'apriltag_0']
    )

    StaticTransformAprilTag1 = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        arguments = ['--x', '1.08', '--y', '-0.7', '--z', '0', '--yaw', '-2.35619', '--pitch', '0.0', '--roll', '1.5708', '--frame-id', 'map', '--child-frame-id', 'apriltag_1']
    )

    StaticTransformAprilTag2 = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        arguments = ['--x', '-1.08', '--y', '-0.7', '--z', '0', '--yaw', '2.35619', '--pitch', '0.0', '--roll', '1.5708', '--frame-id', 'map', '--child-frame-id', 'apriltag_2']
    )


    StaticTransformAprilTag3 = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        arguments = ['--x', '-1.08', '--y', '0.7', '--z', '0', '--yaw', '0.785398', '--pitch', '0.0', '--roll', '1.5708', '--frame-id', 'map', '--child-frame-id', 'apriltag_3']
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ebug_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        StaticTransformAprilTag0,
        StaticTransformAprilTag1,
        StaticTransformAprilTag2,
        StaticTransformAprilTag3,
        Boids,
        Disco
    ])