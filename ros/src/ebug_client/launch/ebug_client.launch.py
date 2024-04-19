import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ROBOT_ID = os.getenv('ROBOT_ID', "default")
    CAMERA_POLLING = os.getenv('CAMERA_POLLING', "disable").lower() == "enable"
    ALL_CAMERAS = os.getenv('ALL_CAMERAS', 'disable').lower() == "enable"

    PKG_SHARE = FindPackageShare(package='ebug_client').find('ebug_client')


    CAMERA_IDS = ['cam_0']
    CAMERA_NODES = []

    CAMERA_NODES.append( create_camera_node(ROBOT_ID, "cam_0", PKG_SHARE, '/dev/video0') )
    if CAMERA_POLLING:   
        CAMERA_IDS.extend( ['cam_1', 'cam_2', 'cam_3'] )
        CAMERA_NODES.append( create_camera_node(ROBOT_ID, "cam_1", PKG_SHARE, '/dev/video1') )
        CAMERA_NODES.append( create_camera_node(ROBOT_ID, "cam_2", PKG_SHARE, '/dev/video2') )
        CAMERA_NODES.append( create_camera_node(ROBOT_ID, "cam_3", PKG_SHARE, '/dev/video3') )
    

    CameraControllerNode = Node(
        package = 'ebug_client',
        executable = 'CameraController',
        name = 'CameraController',
        namespace = ROBOT_ID,
        parameters=[
            {'all_cameras', str(ALL_CAMERAS)    },
            {'cameras':     CAMERA_IDS          },
        ]
    )

    RobotControllerNode = Node(
        package = 'ebug_client',
        executable = 'RobotController',
        name = 'RobotController',
        namespace = ROBOT_ID
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ebug_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        *CAMERA_NODES,
        CameraControllerNode,
        TimerAction(period=5.0, actions=[RobotControllerNode]) # Apply delayed start to movement controller, allow initial localization
    ])



def create_camera_node(ROBOT_ID, CAM_ID, PKG_SHARE, VIDEO_DEVICE):
    FRAME_RATE = 25.0
    WIDTH = 640
    HEIGHT = 480
    CAM_INFO = os.path.join(PKG_SHARE, f'calibration/{CAM_ID}.yaml') 

    return Node(
        package = 'usb_cam',
        executable = 'usb_cam_node_exe',
        name = 'Camera',
        namespace = f'{ROBOT_ID}/{CAM_ID}',

        parameters=[
            {'video_device':    VIDEO_DEVICE        },  
            {'camera_name':     CAM_ID              },
            {'camera_info_url': f'file://{CAM_INFO}'},
            {'frame_id':        CAM_ID              },
            {'pixel_format':    'yuyv'              },
            {'framerate':       FRAME_RATE          },
            {'image_height':    HEIGHT              },
            {'image_width':     WIDTH               },
            
            # {'autofocus':       True                },
        ]
    )