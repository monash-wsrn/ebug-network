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
from launch_ros.substitutions import FindPackageShare  

def generate_launch_description():
    ROBOT_ID = os.getenv('ROBOT_ID', "default")
    ROBOT_ALGO = os.getenv('ROBOT_ALGO', "BoidsService")

    PKG_SHARE = FindPackageShare(package='ebug_agent').find('ebug_agent')
    APRIL_TAG_PATH = os.path.join(PKG_SHARE, 'config/aprilTag.yaml') 
    EKF_POSE_PATH = os.path.join(PKG_SHARE, 'config/ekf.yaml') 


    # launch the image processing nodes
    ImageProcNode = ComposableNode(
        package = 'image_proc',
        plugin = 'image_proc::RectifyNode',
        name = 'RectifyColorNode',
        namespace = ROBOT_ID,

        remappings = [
            ('image', 'image_raw')
        ]
    )

    # launch the apriltag nodes
    AprilTagNode =  Node(
        package = 'apriltag_ros',
        executable = 'apriltag_node',
        name = 'AprilTag',
        namespace = ROBOT_ID,
        
        parameters = [ APRIL_TAG_PATH ],
        remappings = [
            ('tf', 'tf_detections'),
        ]
    )

    # converter node to invert the transform of cam->tag
    TransformConverterNode = Node(
        package = 'ebug_agent',
        executable = 'TransformConverter',
        name = 'TransformConverter',
        namespace = ROBOT_ID,
    )

    # This node is the connector between the central controller and an individual robot.
    MovementControllerNode = Node(
        package = 'ebug_agent',
        executable = 'MovementController',
        name = 'MovementController',
        namespace = ROBOT_ID,

        parameters = [
            {"service_name": ROBOT_ALGO}
        ]
    )


    ContainerLaunchArg = DeclareLaunchArgument(
        name = 'container', 
        default_value = '',
        description = (
            'Name of an existing node container to load launched nodes into. '
            'If unset, a new container will be created.'
        )
    )

    # If an existing container is not provided, start a container and load nodes into it
    ImageProcContainer = ComposableNodeContainer(
        condition = LaunchConfigurationEquals('container', ''),
        name = 'image_proc_container',
        namespace = '', # TODO maybe ROBOT_ID ??
        package = 'rclcpp_components',
        executable = 'component_container',
        composable_node_descriptions = [ ImageProcNode ],
        output = 'screen'
    )

    # If an existing container name is provided, load composable nodes into it
    # This will block until a container with the provided name is available and nodes are loaded
    LoadComposable = LoadComposableNodes(
        condition = LaunchConfigurationNotEquals('container', ''),
        composable_node_descriptions = [ ImageProcNode ],
        target_container = LaunchConfiguration('container'),
    )

    
    
    EKFPose = Node(
        package = 'robot_localization',
        executable = 'ekf_node',
        name = 'ekf_filter_node_pose',
        namespace = ROBOT_ID,

        parameters = [ EKF_POSE_PATH ],
        remappings = [
            ('odometry/filtered', 'filtered_odom'),
        ]
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ebug_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        ContainerLaunchArg,
        ImageProcContainer,
        LoadComposable,
        AprilTagNode,
        TransformConverterNode,
        EKFPose,
        MovementControllerNode
    ])