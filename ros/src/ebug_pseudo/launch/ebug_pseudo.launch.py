import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    ROBOT_ID = os.getenv('ROBOT_ID', "default")
    ROBOT_ALGO = os.getenv('ROBOT_ALGO', "BoidsService")

    
    START_POSX = float(os.getenv('START_POSX', "0.0"))
    START_POSY = float(os.getenv('START_POSY', "0.0"))
    START_YAW = float(os.getenv('START_YAW', "0.0"))
    TICK_RATE = float(os.getenv('TICK_RATE', "25.0"))

    # This node is the connector between the central controller and an individual robot.
    PseudoMovementControllerNode = Node(
        package = 'ebug_pseudo',
        executable = 'PseudoMovementController',
        name = 'PseudoMovementController',
        namespace = ROBOT_ID,

        parameters = [
            {"service_name":    ROBOT_ALGO                  },
            {"tick_rate":       TICK_RATE                   },
            {"start_pos":       [START_POSX, START_POSY]    },
            {"start_yaw":       START_YAW                   }
        ]
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ebug_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        PseudoMovementControllerNode
    ])