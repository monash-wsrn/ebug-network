import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare  

def generate_launch_description():
    ROBOT_ID = "robot_0"        # TODO use os.environ['ROBOT_ID'] instead
    
    #PKG_SHARE = FindPackageShare(package='ebug_client').find('ebug_client')
    #EKF_ODOM_PATH = os.path.join(PKG_SHARE, 'config/ekf_odom.yaml') 
    #EKF_POSE_PATH = os.path.join(PKG_SHARE, 'config/ekf.yaml') 
    

    CameraNode0 = create_camera_node(ROBOT_ID, "cam_0",
        '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0')

    # CameraNode1 = create_camera_node(ROBOT_ID, "cam_1",
    #     '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-video-index0')

    # CameraNode2 = create_camera_node(ROBOT_ID, "cam_2",
    #     '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-video-index0')

    # CameraNode3 = create_camera_node(ROBOT_ID, "cam_3",
    #     '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0')

    CameraPollerNode = Node(
        package = 'ebug_client',
        executable = 'CameraPoller',
        name = f'{ROBOT_ID}_CameraPoller',
        parameters=[
            {'robot_id': ROBOT_ID}
        ]
    )


    RobotControllerNode = Node(
        package = 'ebug_client',
        executable = 'RobotController',
        name = f'{ROBOT_ID}_RobotController',
        parameters=[
            {'robot_id': ROBOT_ID}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ebug_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        CameraNode0,
        #CameraNode1,
        #CameraNode2,
        #CameraNode3,
        CameraPollerNode,
        RobotControllerNode
    ])



def create_camera_node(ROBOT_ID, CAM_ID, VIDEO_DEVICE):
    FRAME_RATE = 10.0
    WIDTH = 640
    HEIGHT = 480

    return Node(
        package = 'usb_cam',
        executable = 'usb_cam_node_exe',
        name = f'{ROBOT_ID}_{CAM_ID}',
        parameters=[
            {'video_device': VIDEO_DEVICE}, # '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0'
            {'camera_name': f'{ROBOT_ID}/{CAM_ID}'},
            {'camera_info_url': f'file:///ws/src/ebug_client/calibration/{CAM_ID}.yaml'},
            {'frame_id': f'{ROBOT_ID}/{CAM_ID}'},
            {'pixel_format':'mjpeg2rgb'},
            {'framerate': FRAME_RATE},
            {'image_height': HEIGHT},
            {'image_width':WIDTH},
        ],
        remappings=[
            # Subscribe
            ('set_capture',f'{ROBOT_ID}/{CAM_ID}/set_capture'),

            # Publish
            ('image_raw', f'{ROBOT_ID}/image_raw'),
            ('camera_info', f'{ROBOT_ID}/camera_info'),
        ]
    )