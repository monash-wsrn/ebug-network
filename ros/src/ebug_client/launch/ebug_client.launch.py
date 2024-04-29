import os
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes

from launch.actions import DeclareLaunchArgument, TimerAction, SetLaunchConfiguration
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ROBOT_ID = os.getenv('ROBOT_ID', "default")
    CAMERA_IDS = os.getenv('CAMERAS', 'cam_0').split(',')

    PKG_SHARE = FindPackageShare(package='ebug_client').find('ebug_client')


    COMPOSABLE_NODES = []
    VIDX = 0    # 0, 2, 4, and 6
    for cam_id in CAMERA_IDS:
        COMPOSABLE_NODES.extend( create_camera_composable_nodes(ROBOT_ID, cam_id, PKG_SHARE, f'/dev/video{VIDX}') )
        VIDX += 2
        
        
    CameraControllerNode = ComposableNode(
        package = 'ebug_cpp',
        plugin = 'ebug::CameraController',
        name = 'CameraController',
        namespace = ROBOT_ID,
        parameters=[
            {'cameras':     CAMERA_IDS  }
        ]
    )
    COMPOSABLE_NODES.append(CameraControllerNode)


    ContainerLaunchArg = DeclareLaunchArgument(
        name=f'{ROBOT_ID}_container', 
        default_value='',
        description=('')
    )

    ComposablesContainer = ComposableNodeContainer(
        condition=LaunchConfigurationEquals(f'{ROBOT_ID}_container', ''),
        package='rclcpp_components',
        executable='component_container',
        name='ebug_composables_container',
        namespace='',
        composable_node_descriptions=COMPOSABLE_NODES,
    )

    ComposablesLoader = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals(f'{ROBOT_ID}_container', ''),
        composable_node_descriptions=COMPOSABLE_NODES,
        target_container=LaunchConfiguration(f'{ROBOT_ID}_container'),
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

        ContainerLaunchArg,
        ComposablesContainer,
        ComposablesLoader,
        TimerAction(period=5.0, actions=[RobotControllerNode]) # Apply delayed start to movement controller, allow initial localization
    ])



def create_camera_composable_nodes(ROBOT_ID, CAM_ID, PKG_SHARE, VIDEO_DEVICE):   
    FRAME_RATE = 10.0
    WIDTH = 640
    HEIGHT = 480
    CAM_INFO = os.path.join(PKG_SHARE, f'calibration/{CAM_ID}.yaml') 
    NAMESPACE = f'{ROBOT_ID}/{CAM_ID}'

    # https://github.com/ros-drivers/usb_cam/blob/ros2/CMakeLists.txt
    CameraNode = ComposableNode(
        package = 'usb_cam',
        plugin = 'usb_cam::UsbCamNode',
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
    ByteRectifier = ComposableNode(
        package = 'ebug_cpp',
        plugin = 'ebug::ByteRectifier',
        name = 'ByteRectifier',
        namespace = NAMESPACE,
    )

    # https://github.com/ros-perception/image_common/blob/rolling/image_transport/CMakeLists.txt
    # Decompress jpeg format into raw image
    MotionJPEGDecompress = ComposableNode(
        package = 'image_transport',
        plugin = 'image_transport::Republisher',
        name = 'DecompressMJPEG',
        namespace = NAMESPACE,
    
        parameters=[
            {'in_transport':    'compressed'    },
            {'out_transport':   'raw'           },
        ],

        remappings = [
            ('in/compressed',   'image_compressed'  ),
            ('out',             'image_uncompressed'), 
        ]
    )

    # https://github.com/ros-perception/image_pipeline/blob/rolling/image_proc/CMakeLists.txt
    # launch the image processing nodes (rectify camera distortion)
    ImageProcNode = ComposableNode(
        package = 'image_proc',
        plugin = 'image_proc::RectifyNode',
        name = 'RectifyColor',
        namespace = NAMESPACE,

        remappings = [
            ('image', 'image_uncompressed')
        ]
    )

    return [CameraNode, ByteRectifier, MotionJPEGDecompress, ImageProcNode]