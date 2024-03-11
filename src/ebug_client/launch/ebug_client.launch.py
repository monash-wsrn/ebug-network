import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare  

def generate_launch_description():
    ROBOT_ID = os.getenv('ROBOT_ID', "default")
    
    PKG_SHARE = FindPackageShare(package='ebug_client').find('ebug_client')
    EKF_ODOM_PATH = os.path.join(PKG_SHARE, 'config/ekf_odom.yaml') 
    EKF_POSE_PATH = os.path.join(PKG_SHARE, 'config/ekf.yaml') 
    

    CameraNode0 = create_camera_node(ROBOT_ID, "cam_0", PKG_SHARE,
        '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0')

    # CameraNode1 = create_camera_node(ROBOT_ID, "cam_1", PKG_SHARE,
    #     '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-video-index0')

    # CameraNode2 = create_camera_node(ROBOT_ID, "cam_2", PKG_SHARE,
    #     '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-video-index0')

    # CameraNode3 = create_camera_node(ROBOT_ID, "cam_3", PKG_SHARE,
    #     '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0')

    CameraPollerNode = Node(
        package = 'ebug_client',
        executable = 'CameraPoller',
        name = 'CameraPoller',
        namespace = ROBOT_ID
    )


    RobotControllerNode = Node(
        package = 'ebug_client',
        executable = 'RobotController',
        name = 'RobotController',
        namespace = ROBOT_ID
    )

    EKFOdom = Node(
        package = 'robot_localization',
        executable = 'ekf_node',
        name = 'ekf_filter_node_odom',
        namespace = ROBOT_ID,

        parameters = [ EKF_ODOM_PATH ],
        remappings = [
            ('odometry/filtered', '/filtered_odom'),
            ('diagnostics', 'diagnostics_odom'), 
        ]
    )
    
    EKFPose = Node(
        package = 'robot_localization',
        executable = 'ekf_node',
        name = 'ekf_filter_node_pose',
        namespace = ROBOT_ID,

        parameters = [ EKF_POSE_PATH ],
        remappings = [
            ('odometry/filtered', '/filtered_pose'),
            ('diagnostics', 'diagnostics_pose'),  
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
        EKFOdom,    # TODO Unsure of filtered_odom usage ??
        EKFPose,
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