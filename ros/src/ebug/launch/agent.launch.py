import os
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes

from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ROBOT_ID = os.getenv('ROBOT_ID', "default")
    ROBOT_ALGO = os.getenv('ROBOT_ALGO', "BoidsService")
    CAMERA_IDS = os.getenv('CAMERAS', 'cam_0').split(',')

    PKG_SHARE = FindPackageShare(package='ebug').find('ebug')


    COMPOSABLE_NODES = []
    VIDX = 0    # 0, 2, 4, and 6
    for cam_id in CAMERA_IDS:
        COMPOSABLE_NODES.extend( create_camera_composable_nodes(ROBOT_ID, cam_id, PKG_SHARE, f'/dev/video{VIDX}') )
        VIDX += 2

    

    # converter node to invert the transform of cam->tag
    Transformer = Node(
        package = 'ebug',
        executable = 'TransformConverter',
        name = 'TransformConverter',
        namespace = ROBOT_ID,
    )

    # https://answers.ros.org/question/222970/fusing-absolute-position-information-and-imu-data-using-the-ekf_localization_node/
    # EKF Node taking in AprilTag detections and wheel odometry
    EKFAbsolute = Node(
        package = 'robot_localization',
        executable = 'ekf_node',
        name = 'ekf_filter_absolute',
        namespace = ROBOT_ID,

        parameters = [ 
            os.path.join(PKG_SHARE, 'config/ekfAbsolute.yaml'),
            {"odom_frame":      f"{ROBOT_ID}_odom"  },
            {"base_link_frame": f"{ROBOT_ID}"       },
        ],
        remappings = [
            ('odometry/filtered', 'ekf_absolute'),
        ]
    )

    
    # EKF Node taking in wheel odometry and IMU readings
    EKFRelative = Node(
        package = 'robot_localization',
        executable = 'ekf_node',
        name = 'ekf_filter_relative',
        namespace = ROBOT_ID,

        parameters = [ 
            os.path.join(PKG_SHARE, 'config/ekfRelative.yaml'),
            {"odom_frame":      f"{ROBOT_ID}_odom"  },
            {"base_link_frame": f"{ROBOT_ID}"       },
            {"world_frame":     f"{ROBOT_ID}_odom"  },
        ],
        remappings = [
            ('odometry/filtered', 'ekf_relative'),
        ]
    )




    # This node takes the velocity and color commands and sends it across I2C
    RobotControllerNode = Node(
        package = 'ebug',
        executable = 'RobotController',
        name = 'RobotController',
        namespace = ROBOT_ID
    )

    # This node is the connector between the central controller and an individual robot.
    MovementControllerNode = Node(
        package = 'ebug',
        executable = 'MovementController',
        name = 'MovementController',
        namespace = ROBOT_ID,

        parameters = [
            {"service_name": ROBOT_ALGO}
        ]
    )



    ContainerLaunchArg = DeclareLaunchArgument(
        name = f'{ROBOT_ID}_container', 
        default_value = '',
        description = ('')
    )

    ComposablesContainer = ComposableNodeContainer(
        condition = LaunchConfigurationEquals(f'{ROBOT_ID}_container', ''),
        package = 'rclcpp_components',
        executable = 'component_container',
        name = 'ebug_composables_container',
        namespace = '',
        composable_node_descriptions = COMPOSABLE_NODES,
    )

    ComposablesLoader = LoadComposableNodes(
        condition = LaunchConfigurationNotEquals(f'{ROBOT_ID}_container', ''),
        composable_node_descriptions = COMPOSABLE_NODES,
        target_container = LaunchConfiguration(f'{ROBOT_ID}_container'),
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ebug_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        ContainerLaunchArg,
        ComposablesContainer,
        ComposablesLoader,
        
        Transformer,
        EKFAbsolute,
        EKFRelative,
        MovementControllerNode,
        RobotControllerNode
    ])



def create_camera_composable_nodes(ROBOT_ID, CAM_ID, PKG_SHARE, VIDEO_DEVICE):   
    FRAME_RATE = 30.0
    WIDTH = 640
    HEIGHT = 480
    CAM_INFO = os.path.join(PKG_SHARE, f'calibration/{CAM_ID}.yaml') 
    APRIL_TAG_PATH = os.path.join(PKG_SHARE, 'config/aprilTag.yaml') 
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
        package = 'ebug_base',
        plugin = 'ebug::ByteRectifier',
        name = 'ByteRectifier',
        namespace = NAMESPACE,
    )

    # https://github.com/ros-perception/image_common/blob/rolling/image_transport/CMakeLists.txt
    # Decompress jpeg format into raw image
    MotionJPEGDecompress = ComposableNode(
        package = 'ebug_base',
        plugin = 'ebug::JpegRepublisher',
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

    # https://github.com/christianrauch/apriltag_ros/blob/master/CMakeLists.txt
    # launch the apriltag node
    AprilTagNode = ComposableNode(
        package = 'apriltag_ros',
        plugin = 'AprilTagNode',
        name = 'AprilTag',
        namespace = NAMESPACE,
        
        parameters = [ APRIL_TAG_PATH ],
        remappings = [
            ('/tf', f'/{ROBOT_ID}/tf_detections'),  # This must be '/tf' as the AprilTag publishes to the absolute topic
                                                    # We also want to publish absolutely to /robot_id/tf_detections
        ]
    )

    return [CameraNode, ByteRectifier, MotionJPEGDecompress, ImageProcNode, AprilTagNode]