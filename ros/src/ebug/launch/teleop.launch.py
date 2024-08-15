import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ROBOT_ID = os.getenv('ROBOT_ID', "default")
    PKG_SHARE = FindPackageShare(package='ebug').find('ebug')

    return LaunchDescription([
        # Launch the RobotController node
        Node(
            package='ebug',
            executable='RobotController',
            name='RobotController',
            namespace=ROBOT_ID,
            output='screen'
        ),
        Node(
            package='ebug',
            executable='tf_publisher',
            name='tf_publisher',
            output='screen'
        ),

        # EKF Node taking in wheel odometry and IMU readings
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_relative',
            namespace=ROBOT_ID,
            parameters=[
                os.path.join(PKG_SHARE, 'config/ekfRelative.yaml'),  # Path to the EKF configuration file
                {"odom_frame": f"{ROBOT_ID}_odom"},
                {"base_link_frame": f"{ROBOT_ID}"},
                {"world_frame": f"{ROBOT_ID}_odom"},
            ],
            remappings=[
                ('odometry/filtered', 'ekf_relative'),
            ]
        )
    ])
