import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare  

def generate_launch_description():
    ROBOT_ID = os.getenv('ROBOT_ID', "default")
    CAMERA_POLLING = os.getenv('CAMERA_POLLING', "disable").lower() == "enable"

    PKG_SHARE = FindPackageShare(package='ebug_client').find('ebug_client')

    CameraNode0 = create_camera_node(ROBOT_ID, "cam_0", PKG_SHARE, '/dev/video0')
    
    
    RobotControllerNode = Node(
        package = 'ebug_client',
        executable = 'RobotController',
        name = 'RobotController',
        namespace = ROBOT_ID
    )

    # If camera polling is disabled (default), use only the one camera
    if not CAMERA_POLLING:
        return LaunchDescription([
            DeclareLaunchArgument(
                'use_ebug_time',
                default_value='false',
                description='Use simulation (Gazebo) clock if true'),

            CameraNode0,
            TimerAction(period=10.0, actions=[RobotControllerNode]) # Apply delayed start to movement controller, allow initial localization
        ])
    



    # If camera polling is enabled, initialize the other camera(s) and CameraPoller
    CameraNode1 = create_camera_node(ROBOT_ID, "cam_1", PKG_SHARE, '/dev/video1')

    CameraNode2 = create_camera_node(ROBOT_ID, "cam_2", PKG_SHARE, '/dev/video2')

    CameraNode3 = create_camera_node(ROBOT_ID, "cam_3", PKG_SHARE, '/dev/video3')

    CameraPollerNode = Node(
        package = 'ebug_client',
        executable = 'CameraPoller',
        name = 'CameraPoller',
        namespace = ROBOT_ID
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ebug_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        CameraNode0,
        CameraNode1,
        CameraNode2,
        CameraNode3,
        CameraPollerNode,
        TimerAction(period=10.0, actions=[RobotControllerNode]) # Apply delayed start to movement controller, allow initial localization
    ])



def create_camera_node(ROBOT_ID, CAM_ID, PKG_SHARE, VIDEO_DEVICE):
    FRAME_RATE = 10.0
    WIDTH = 640
    HEIGHT = 480
    CAM_INFO = os.path.join(PKG_SHARE, f'calibration/{CAM_ID}.yaml') 

    return Node(
        package = 'usb_cam',
        executable = 'usb_cam_node_exe',
        name = CAM_ID,
        namespace = ROBOT_ID,

        parameters=[
            {'video_device':    VIDEO_DEVICE        },
            {'camera_name':     CAM_ID              },
            {'camera_info_url': f'file://{CAM_INFO}'},
            {'frame_id':        CAM_ID              },
            {'pixel_format':    'mjpeg2rgb'         },
            {'framerate':       FRAME_RATE          },
            {'image_height':    HEIGHT              },
            {'image_width':     WIDTH               },
        ],
        remappings=[
            # Subscribe
            ('set_capture', f'{CAM_ID}/set_capture'),
        ]
    )