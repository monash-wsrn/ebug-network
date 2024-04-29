import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ROBOT_ID = os.getenv('ROBOT_ID', "default")
    CAMERA_IDS = os.getenv('ALL_CAMERAS', 'cam_0').split(',')

    PKG_SHARE = FindPackageShare(package='ebug_client').find('ebug_client')


    CAMERA_NODES = []
    VIDX = 0    # 0, 2, 4, and 6
    for cam_id in CAMERA_IDS:
        CAMERA_NODES.extend( create_camera_nodes(ROBOT_ID, cam_id, PKG_SHARE, f'/dev/video{VIDX}') )
        VIDX += 2
        
        
    CameraControllerNode = Node(
        package = 'ebug_client',
        executable = 'CameraController',
        name = 'CameraController',
        namespace = ROBOT_ID,
        parameters=[
            {'cameras':     CAMERA_IDS  }
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



def create_camera_nodes(ROBOT_ID, CAM_ID, PKG_SHARE, VIDEO_DEVICE):   
    FRAME_RATE = 10.0
    WIDTH = 640
    HEIGHT = 480
    CAM_INFO = os.path.join(PKG_SHARE, f'calibration/{CAM_ID}.yaml') 
    NAMESPACE = f'{ROBOT_ID}/{CAM_ID}'

    CameraNode = Node(
        package = 'usb_cam',
        executable = 'usb_cam_node_exe',
        name = 'Camera',
        namespace = NAMESPACE,

        parameters=[
            {'video_device':    VIDEO_DEVICE        },  
            {'camera_name':     CAM_ID              },
            {'camera_info_url': f'file://{CAM_INFO}'},
            {'frame_id':        CAM_ID              },
            {'pixel_format':    'raw_mjpeg'         },
            {'framerate':       FRAME_RATE          },
            {'image_height':    HEIGHT              },
            {'image_width':     WIDTH               },
        ]
    )

    # Correctly select raw camera bytes for compressed data stream
    ByteRectifier = Node(
        package = 'ebug_client',
        executable = 'ByteRectifier',
        name = 'ByteRectifier',
        namespace = NAMESPACE,
    )

    # Decompress jpeg format into raw image
    MotionJPEGDecompress = Node(
        package = 'image_transport',
        executable = 'republish',
        name = 'DecompressMJPEG',
        namespace = NAMESPACE,
    
        arguments = [
            'compressed',
            'raw',
        ],
        remappings = [
            ('in/compressed', 'image_compressed'),
            ('out', 'image_uncompressed'), 
        ]
    )

    # launch the image processing nodes (rectify camera distortion)
    ImageProcNode = Node(
        package = 'image_proc',
        executable = 'rectify_node',
        name = 'RectifyColor',
        namespace = NAMESPACE,

        remappings = [
            ('image', 'image_uncompressed')
        ]
    )

    return [CameraNode, ByteRectifier, MotionJPEGDecompress, ImageProcNode]