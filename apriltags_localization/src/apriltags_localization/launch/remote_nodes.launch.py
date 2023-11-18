from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable   
import os

pkg_share = FindPackageShare(package='apriltags_localization').find('apriltags_localization')

robot_localization_file_path = os.path.join(pkg_share, 'config/ekf_odom.yaml') 
ekf_tags_file_path = os.path.join(pkg_share, 'config/ekf.yaml') 

def generate_launch_description():
    return LaunchDescription([

        # # launch the ekf node

        
        ### static frames for the apriltags in respect to the centre of beacon (0,0,0) world frame
        # based on https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0.11', '--y', '0.0', '--z', '0', '--yaw', '1.5708', '--pitch', '0', '--roll', '1.5708', '--frame-id', 'map', '--child-frame-id', 'apriltag_0']
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

      
      

        # ExecuteProcess(
        #         cmd=[[
        #         FindExecutable(name='ros2'),
        #         ' run tf2_ros tf2_echo apriltag_0 robot_0 > robot_pose.csv',


        #         ]],
        #         shell=True
        # ),

        # ExecuteProcess(
        #         cmd=[[
        #         FindExecutable(name='ros2'),
        #         ' run tf2_ros tf2_echo apriltag_0 odom > odom_pose.csv',


        #         ]],
        #         shell=True
        # ),

        
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['--x', '0.0', '--y', '0.0', '--z', '0', '--yaw', '0.0', '--pitch', '0', '--roll', '0', '--frame-id', 'apriltag_2', '--child-frame-id', '2']
        # ),
       
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['--x', '0.386', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'apriltag_0']
        # ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['--x', '0.27294', '--y', '0.27294', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0.7854', '--frame-id', 'world', '--child-frame-id', 'apriltag_1']
        # ),
        
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['--x', '0', '--y', '0.386', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '1.5708', '--frame-id', 'world', '--child-frame-id', 'apriltag_2']
        # ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['--x', '-0.27294', '--y', '0.27294', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '2.3562', '--frame-id', 'world', '--child-frame-id', 'apriltag_3']
        # ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['--x', '-0.386', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '3.1416', '--frame-id', 'world', '--child-frame-id', 'apriltag_4']
        # ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['--x', '-0.27294', '--y', '-0.27284', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '-2.3562', '--frame-id', 'world', '--child-frame-id', 'apriltag_5']
        # ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['--x', '0', '--y', '-0.386', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '-1.5708', '--frame-id', 'world', '--child-frame-id', 'apriltag_6']
        # ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['--x', '0.27294', '--y', '-0.27294', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '-0.7854', '--frame-id', 'world', '--child-frame-id', 'apriltag_7']
        # )
    ])