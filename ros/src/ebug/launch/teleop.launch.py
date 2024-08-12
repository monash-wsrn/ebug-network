import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ROBOT_ID = os.getenv('ROBOT_ID', "default")

    return LaunchDescription([
        # Launch the RobotController node
        Node(
            package='ebug',
            executable='RobotController',
            name='RobotController',
            namespace=ROBOT_ID,
            output='screen'
        ),

        # Add other nodes here as necessary
        # Example: Launch another node if needed
        # Node(
        #     package='another_package',
        #     executable='another_node',
        #     name='another_node',
        #     namespace=ROBOT_ID,
        #     output='screen'
        # )
    ])
