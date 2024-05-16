import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ROBOT_ID = os.getenv('ROBOT_ID', "default")
    use_sim_robot = os.getenv('USE_SIM_ROBOT', "false").lower() == "true"
    use_keyboard = os.getenv('USE_KEYBOARD', "false").lower() == "true"

    nodes = []

    # Add the sim_robot node if USE_SIM_ROBOT is true
    if use_sim_robot:
        nodes.append(Node(
            package='ebug',
            executable='sim_robot',
            name='SimRobot',
            namespace=ROBOT_ID,
        ))
    else: 
        nodes.append(Node(
            package='ebug',
            executable='TeleoperationController',
            name='TeleoperationController',
            namespace=ROBOT_ID,
            output='screen'
        ))

    # Add the teleoperation node if USE_KEYBOARD is true
    if use_keyboard:
        nodes.append(Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            remappings=[
                ('/cmd_vel', f'/{ROBOT_ID}/cmd_vel')
            ]
        ))

    return LaunchDescription(nodes)
