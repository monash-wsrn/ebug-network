import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    ROBOT_ID = "robot_0"        # TODO use os.environ['ROBOT_ID'] instead
    ROBOT_ALGO = "BoidsService" # TODO use os.environ['ROBOT_ALGO'] instead


    # launch the image processing nodes
    ImageProcNode = ComposableNode(
        package = 'image_proc',
        plugin = 'image_proc::RectifyNode',
        name=f'{ROBOT_ID}_RectifyColorNode',
        # Remap subscribers and publishers
        remappings=[
            # Subscribe
            ('image',       f'{ROBOT_ID}/image_raw'),
            ('camera_info', f'{ROBOT_ID}/camera_info'),

            # Publish
            ('image_rect',  f'{ROBOT_ID}/image_rect')
        ],
    )

    # launch the apriltag nodes
    AprilTagNode =  Node(
        package = 'apriltag_ros',
        executable = 'apriltag_node',
        name=f'{ROBOT_ID}_AprilTag',
        
        parameters=[ '/home/ubuntu/networked_robotics/ros2_localization/src/localization/config/aprilTag.yaml' ],

        remappings=[
            # Subscrube
            ('camera_info', f'{ROBOT_ID}/camera_info'),
            ('image_rect',  f'{ROBOT_ID}/image_rect'),

            # Publish
            ('tf',          f'{ROBOT_ID}/tf_detections'),
            ('detections',  f'{ROBOT_ID}/detections')
        ]
        
    )

        # converter node to invert the transform of cam->tag
    TransformConverterNode = Node(
        package='ebug_agent',
        executable='TransformConverter',
        name=f'{ROBOT_ID}_TransformConverter',
        parameters=[
            {'robot_id': ROBOT_ID}
        ]
    )

    # This node is the connector between the central controller and an individual robot.
    # Simply duplicate this node and set the robot_id parameter appropriately
    MovementControllerNode = Node(
        package='ebug_principal',
        executable='RobotController',
        name=f'{ROBOT_ID}_RobotController',
        parameters=[
            {"robot_id": ROBOT_ID},
            {"service_name": ROBOT_ALGO}
        ]
    )


    # If an existing container is not provided, start a container and load nodes into it
    ImageProcContainer = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', ''),
        name='image_proc_container',
        namespace='',   # TODO requires ROBOT_ID ??
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[ImageProcNode],
        output='screen'
    )

    # If an existing container name is provided, load composable nodes into it
    # This will block until a container with the provided name is available and nodes are loaded
    LoadComposable = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals('container', ''),
        composable_node_descriptions=[ImageProcNode],
        target_container=LaunchConfiguration('container'),
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ebug_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        ImageProcContainer,
        LoadComposable,
        AprilTagNode,
        TransformConverterNode,
        MovementControllerNode
    ])