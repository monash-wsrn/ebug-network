import os
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def create_camera_composable_nodes(ROBOT_ID, CAM_ID, PKG_SHARE, VIDEO_DEVICE):
    FRAME_RATE = 10.0
    WIDTH = 320
    HEIGHT = 240
    CAM_INFO = os.path.join(PKG_SHARE, f'calibration/{CAM_ID}.yaml')
    APRIL_TAG_PATH = os.path.join(PKG_SHARE, 'config/aprilTag.yaml')
    NAMESPACE = f'{ROBOT_ID}/{CAM_ID}'

    nodes = [
        ComposableNode(
            package='usb_cam', plugin='usb_cam::UsbCamNode', name='Camera',
            namespace=NAMESPACE,
            parameters=[
                {'video_device': VIDEO_DEVICE, 'camera_name': CAM_ID,
                 'camera_info_url': f'file://{CAM_INFO}', 'frame_id': CAM_ID,
                 'pixel_format': 'raw_mjpeg', 'framerate': FRAME_RATE,
                 'image_height': HEIGHT, 'image_width': WIDTH}
            ]
        ),
        ComposableNode(
            package='ebug_base', plugin='ebug::ByteRectifier', 
            name='ByteRectifier', namespace=NAMESPACE
        ),
        ComposableNode(
            package='ebug_base', plugin='ebug::JpegRepublisher', 
            name='DecompressMJPEG', namespace=NAMESPACE,
            parameters=[{'in_transport': 'compressed', 'out_transport': 'raw'}],
            remappings=[('in/compressed', 'image_compressed'), 
                        ('out', 'image_uncompressed')]
        ),
        ComposableNode(
            package='image_proc', plugin='image_proc::RectifyNode', 
            name='RectifyColor', namespace=NAMESPACE,
            remappings=[('image', 'image_uncompressed')]
        ),
        ComposableNode(
            package='apriltag_ros', plugin='AprilTagNode', 
            name='AprilTag', namespace=NAMESPACE,
            parameters=[APRIL_TAG_PATH],
            remappings=[('/tf', f'/{ROBOT_ID}/tf_detections')]
        )
    ]
    return nodes

def create_robot_nodes(ROBOT_ID, PKG_SHARE):
    return [
        Node(
            package='ebug', executable='RobotController',
            name='RobotController', namespace=ROBOT_ID, output='screen'
        ),
        Node(
            package='ebug',
            executable='TransformConverter',
            name='TransformConverter',
            namespace='ebug03',
            output='screen'
        ),
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_relative', namespace=ROBOT_ID,
            parameters=[
                os.path.join(PKG_SHARE, 'config/ekfRelative.yaml'),
                {"odom_frame": f"{ROBOT_ID}_odom"},
                {"base_link_frame": f"{ROBOT_ID}"},
                {"world_frame": f"{ROBOT_ID}_odom"},
            ],
            remappings=[('odometry/filtered', 'ekf_relative')]
        ),
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_absolute', namespace=ROBOT_ID,
            parameters=[
                os.path.join(PKG_SHARE, 'config/ekfAbsolute.yaml'),
                {"odom_frame": f"{ROBOT_ID}_odom"},
                {"base_link_frame": f"{ROBOT_ID}"},
                {"world_frame": "map"},  # Assuming you're using "map" as the world frame
            ],
            remappings=[
                ('odometry/filtered', 'ekf_absolute'),
                ('/tf', '/ebug03/tf_detections'),
                ('pose0', '/ebug03/apriltag_pose')
            ]
        )
    ]

def generate_launch_description():
    ROBOT_ID = os.getenv('ROBOT_ID', "default")
    CAM_ID = 'cam_1'
    PKG_SHARE = FindPackageShare(package='ebug').find('ebug')
    VIDEO_DEVICE = '/dev/video2'

    camera_nodes = create_camera_composable_nodes(ROBOT_ID, CAM_ID, PKG_SHARE, VIDEO_DEVICE)
    robot_nodes = create_robot_nodes(ROBOT_ID, PKG_SHARE)

    return LaunchDescription([
        GroupAction(
            actions=[
                ComposableNodeContainer(
                    name='camera_container',
                    namespace='',
                    package='rclcpp_components',
                    executable='component_container',
                    composable_node_descriptions=camera_nodes,
                    output='screen',
                ),
                *robot_nodes
            ]
        )
    ])
