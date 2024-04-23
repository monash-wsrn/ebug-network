import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare  

def generate_launch_description():
    ROBOT_ID = os.getenv('ROBOT_ID', "default")
    ROBOT_ALGO = os.getenv('ROBOT_ALGO', "BoidsService")

    PKG_SHARE = FindPackageShare(package='ebug_agent').find('ebug_agent')
    APRIL_TAG_PATH = os.path.join(PKG_SHARE, 'config/aprilTag.yaml') 
    EKF_POSE_PATH = os.path.join(PKG_SHARE, 'config/ekf.yaml') 

    # launch mjpeg decompression node
    MotionJPEGDecompress = Node(
        package = 'image_transport',
        executable = 'republish',
        name = 'DecompressMJPEG',
        namespace = ROBOT_ID,
    
        arguments = [
            'compressed',
            'in:=image_raw/compressed',
            'raw',
            'out:=image_raw/uncompressed',
        ]
    )

    # launch the image processing nodes
    ImageProcNode = Node(
        package = 'image_proc',
        executable = 'rectify_node',
        name = 'RectifyColor',
        namespace = ROBOT_ID,

        remappings = [
            ('image', 'image_raw/uncompressed')
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
            ('/tf', 'tf_detections'),   # This must be '/tf' as the AprilTag publishes to the absolute topic
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
        
        ImageProcNode,
        AprilTagNode,
        TransformConverterNode,
        EKFPose,
        MovementControllerNode
    ])