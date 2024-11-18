import os
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
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
                 'camera_info_url': f'file://{CAM_INFO}', 'frame_id': f'{ROBOT_ID}/{CAM_ID}', 
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


def generate_launch_description():
    ROBOT_ID = os.getenv('ROBOT_ID', "default")
    CAMERA_IDS = os.getenv('CAMERAS', 'cam_0').split(',')
    PKG_SHARE = FindPackageShare(package='ebug').find('ebug')

    # Create camera nodes for all cameras
    COMPOSABLE_NODES = []
    camera_transforms = []
    VIDX = 0  # Video device index
    for cam_id in CAMERA_IDS:
        COMPOSABLE_NODES.extend(
            create_camera_composable_nodes(
                ROBOT_ID,
                cam_id,
                PKG_SHARE,
                f'/dev/video{VIDX}'
            )
        )
        camera_transforms.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['0', '0', '0.1', '0', '0', '0', 
                          f'{ROBOT_ID}', f'{ROBOT_ID}/{cam_id}'],
                name=f'static_transform_base_to_{cam_id}'
            )
        )
        VIDX += 2

    robot_nodes = [
        Node(
            package='ebug', executable='RobotController',
            name='RobotController', namespace=ROBOT_ID, output='screen'
        ),
        Node(
            package='ebug', executable='TransformConverter',
            name='TransformConverter', namespace=ROBOT_ID, output='screen', remappings=[
                ('tf_detections', f'/{ROBOT_ID}/tf_detections'),
                ('pose', f'/{ROBOT_ID}/apriltag_pose')]
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
                {"world_frame": "map"},
            ],
            remappings=[
                ('odometry/filtered', 'ekf_absolute'),
                ('/tf', f'/{ROBOT_ID}/tf_detections'),
                ('odom0', f'/{ROBOT_ID}/ekf_relative'),  # Use ekf_relative as odom input
                ('pose0', f'/{ROBOT_ID}/apriltag_pose')  # Use AprilTag pose as pose input
            ]
        ),
        # Add a static transform between map and ebug03/odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', f'{ROBOT_ID}_odom'],  # Adjust offset if necessary
            name='static_transform_map_to_odom'
        ), 
    ]
    
    robot_nodes.extend(camera_transforms)
    
    return LaunchDescription([
        GroupAction(
            actions=[
                ComposableNodeContainer(
                    name='camera_container',
                    namespace='',
                    package='rclcpp_components',
                    executable='component_container',
                    composable_node_descriptions=COMPOSABLE_NODES,
                    output='screen',
                ),
                *robot_nodes
            ]
        )
    ])
