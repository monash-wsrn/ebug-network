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

        # Launch the teleop_twist_keyboard node
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            remappings=[
                ('/cmd_vel', f'/{ROBOT_ID}/cmd_vel')
            ]
        )
    ])
