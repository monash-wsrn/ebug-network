from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessStart, OnExecutionComplete
from launch.substitutions import FindExecutable     
from launch_ros.substitutions import FindPackageShare    
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import os

ROBOT_ID = '0'
pkg_share = FindPackageShare(package='localization').find('localization')

ekf_odom_path = os.path.join(pkg_share, 'config/ekf_odom.yaml') 
ekf_pose_path = os.path.join(pkg_share, 'config/ekf.yaml') 

def generate_launch_description():

    # launch the apriltag nodes
    aprilTag =  Node(
        package = 'apriltag_ros',
        executable = 'apriltag_node',
        name = 'robot_'+ROBOT_ID+'_aprilTag',
        
        parameters=[
            '/home/ubuntu/networked_robotics/ros2_localization/src/localization/config/aprilTag.yaml',
            
        ],

        remappings=[
            ('image_rect', 'robot_'+ROBOT_ID+'/image_rect'),
            ('tf', 'robot_'+ROBOT_ID+'/tf_detections'),
            ('camera_info', 'robot_'+ROBOT_ID+'/camera_info'),
            ('detections', 'robot_'+ROBOT_ID+'/detections')
        ]
        
    )

    # laucnh the image processing nodes
    image_proc = ComposableNode(
        package = 'image_proc',
        plugin = 'image_proc::RectifyNode',
        name='rectify_color_node',
        # Remap subscribers and publishers
        remappings=[
            ('image', 'robot_'+ROBOT_ID+'/image_raw'),
            ('camera_info', 'robot_'+ROBOT_ID+'/camera_info'),
            ('image_rect', 'robot_'+ROBOT_ID+'/image_rect')
        ],

    )

    # converter node to invert the transform of cam->tag
    converter = Node(
        package='localization',
        executable='converter',
        name= 'robot_'+ROBOT_ID+'_converter',
        parameters=[
            {'robot_id': ROBOT_ID}
        ]
    )

    arg_container = DeclareLaunchArgument(
    name='container', default_value='',
    description=(
        'Name of an existing node container to load launched nodes into. '
        'If unset, a new container will be created.'
        )
    )

    # If an existing container is not provided, start a container and load nodes into it
    image_processing_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', ''),
        name='image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[image_proc],
        output='screen'
    )

    # If an existing container name is provided, load composable nodes into it
    # This will block until a container with the provided name is available and nodes are loaded
    load_composable_nodes = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals('container', ''),
        composable_node_descriptions=[image_proc],
        target_container=LaunchConfiguration('container'),
    )

    # launch the cameras
    cam_node_0 =  Node(
        package = 'usb_cam',
        executable = 'usb_cam_node_exe',
        name = 'robot_'+ROBOT_ID+'_cam_0',
        parameters=[
            {'video_device': '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0'},
            {'camera_name': 'robot_'+ROBOT_ID+'/cam_0'},
            {'camera_info_url': 'file:///home/ubuntu/networked_robotics/calibration/cam_0.yaml'},
            {'frame_id': 'robot_'+ROBOT_ID+"/cam_0"},
            {'pixel_format':'mjpeg2rgb'},
            {'framerate': 10.0},
            {'image_height': 480},
            {'image_width':640},
        ],
        remappings=[
            ('image_raw', 'robot_'+ROBOT_ID+'/image_raw'),
            ('set_capture','robot_'+ROBOT_ID+'/cam_0/set_capture'),
            ('camera_info', 'robot_'+ROBOT_ID+'/camera_info'),
        ]
    )

    ########################### (Used only for 4 cameras) ########################### 
    # cam_node_1 =  Node(
    #     package = 'usb_cam',
    #     executable = 'usb_cam_node_exe',
    #     name = 'robot_'+ROBOT_ID+'_cam_1',
        
    #     parameters=[
    #         {'video_device': '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-video-index0'},
    #         {'camera_name': 'robot_'+ROBOT_ID+'/cam_1'},
    #         {'camera_info_url': 'file:///home/ubuntu/github_repo/networked_robotics/calibration/cam_1.yaml'},
    #         {'frame_id': 'robot_'+ROBOT_ID+"/cam_1"},
    #         {'framerate': 25.0},
    #         {'pixel_format':'mjpeg2rgb'},
    #         {'image_height': 480},
    #         {'image_width':640},
    #     ],
    #     remappings=[
    #         ('image_raw', 'robot_'+ROBOT_ID+'/image_raw'),
    #         ('set_capture','robot_'+ROBOT_ID+'/cam_1/set_capture'),
    #         ('camera_info', 'robot_'+ROBOT_ID+'/camera_info'),
    #     ]
    # )

    # cam_node_2 =  Node(
    #     package = 'usb_cam',
    #     executable = 'usb_cam_node_exe',
    #     name = 'robot_'+ROBOT_ID+'_cam_2',
        
    #     parameters=[
    #         {'video_device': '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-video-index0'},
    #         {'camera_name': 'robot_'+ROBOT_ID+'/cam_2'},
    #         {'camera_info_url': 'file:///home/ubuntu/github_repo/networked_robotics/calibration/cam_2.yaml'},
    #         {'frame_id': 'robot_'+ROBOT_ID+"/cam_2"},
    #         {'framerate': 25.0},
    #         {'pixel_format':'mjpeg2rgb'},
    #         {'image_height': 480},
    #         {'image_width':640},
    #     ],
    #     remappings=[
    #         ('image_raw', 'robot_'+ROBOT_ID+'/image_raw'),
    #         ('set_capture','robot_'+ROBOT_ID+'/cam_2/set_capture'),
    #         ('camera_info', 'robot_'+ROBOT_ID+'/camera_info'),
    #     ]
    # )

    # cam_node_3 =  Node(
    #     package = 'usb_cam',
    #     executable = 'usb_cam_node_exe',
    #     name = 'robot_'+ROBOT_ID+'_cam_3',
        
    #     parameters=[
    #         {'video_device': '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0'},
    #         {'camera_name': 'robot_'+ROBOT_ID+'/cam_3'},
    #         {'camera_info_url': 'file:///home/ubuntu/github_repo/networked_robotics/calibration/cam_3.yaml'},
    #         {'frame_id': 'robot_'+ROBOT_ID+"/cam_3"},
    #         {'framerate': 25.0},
    #         {'pixel_format':'mjpeg2rgb'},
    #         {'image_height': 480},
    #         {'image_width':640},
    #     ],
    #     remappings=[
    #         ('image_raw', 'robot_'+ROBOT_ID+'/image_raw'),
    #         ('set_capture','robot_'+ROBOT_ID+'/cam_3/set_capture'),
    #         ('camera_info', 'robot_'+ROBOT_ID+'/camera_info'),
    #     ]
    # )
    

    # the robot PID controller node
    robot = Node(
        package = 'control',
        executable = 'robot',
        name = 'robot_'+ROBOT_ID,
        parameters=[
            {'robot_id': ROBOT_ID}
        ]
    )

    # launch the poller node
    poller = Node(
        package = 'localization',
        executable = 'poller',
        name = 'robot_'+ROBOT_ID+'_poller',
        parameters=[
            {'robot_id': ROBOT_ID}
        ]
    )

    # node for odometry
    ekf_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        parameters=[   
            ekf_odom_path
        ],
        remappings=[
            ('/odometry/filtered', '/robot_0/filtered_odom'),
            ('diagnostics', 'diagnostics_odom'), 
        ]
    )

    # node for pose (tag detection)    
    
    ekf_pose = Node(

        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_pose',
        parameters=[
            ekf_pose_path
        ],
        remappings=[
            ('/odometry/filtered', '/robot_0/filtered_pose'),
            ('diagnostics', 'diagnostics_pose'),  
        ]
    )

    ########################### (Used only for 4 cameras) ########################### 
    # ## event handlers

    # ## close the camera once it is started to prevent bandwidth issues
    # close_cam_0 = ExecuteProcess(
    #         cmd=[[
    #         FindExecutable(name='ros2'),
    #         ' service call ',
    #         'robot_'+ROBOT_ID+'/cam_0/set_capture',
    #         ' std_srvs/srv/SetBool',
    #         ' "{data : False}" '

    #         ]],
    #         shell=True
    # )

    # close_cam_1 = ExecuteProcess(
    #         cmd=[[
    #         FindExecutable(name='ros2'),
    #         ' service call ',
    #         'robot_'+ROBOT_ID+'/cam_1/set_capture ',
    #         ' std_srvs/srv/SetBool ',
    #         ' "{data : False}" '

    #         ]],
    #         shell=True
    # )

    # close_cam_2 = ExecuteProcess(
    #         cmd=[[
    #         FindExecutable(name='ros2'),
    #         ' service call ',
    #         'robot_'+ROBOT_ID+'/cam_2/set_capture ',
    #         ' std_srvs/srv/SetBool ',
    #         '  "{data : False}" '

    #         ]],
    #         shell=True
    # )

    # close_cam_3 = ExecuteProcess(
    #         cmd=[[
    #         FindExecutable(name='ros2'),
    #         ' service call ',
    #         'robot_'+ROBOT_ID+'/cam_3/set_capture ',
    #         ' std_srvs/srv/SetBool ',
    #         '  "{data : False}" '

    #         ]],
    #         shell=True
    # )


    return LaunchDescription([

        ## static frame for base_link to cameras
        # launch the remainings node

        ########################### (Used only for 4 cameras) ########################### 
        # RegisterEventHandler(
        #     OnProcessStart(
        #         target_action=cam_node_0,
        #         on_start=[
        #             # might be not needed anymore, but if these are launch altogether and caused SSH issue, then it might need to be kept
        #             TimerAction(
        #                 period=3.0,
        #                 actions=[cam_node_1],
        #             ),
        #             TimerAction(
        #                 period=3.0,
        #                 actions=[cam_node_2],
        #             ),
        #             TimerAction(
        #                 period=3.0,
        #                 actions=[cam_node_3],
        #             )
        #         ]
        #     )
        # ),


        # RegisterEventHandler(
        #     OnProcessStart(
        #         target_action=cam_node_0,
        #         on_start=[
        #             close_cam_0
        #         ]
        #     )
        # ),
        
        # RegisterEventHandler(
        #     OnProcessStart(
        #         target_action=cam_node_1,
        #         on_start=[
        #             close_cam_1
        #         ]
        #     )
        # ),


        # RegisterEventHandler(
        #     OnProcessStart(
        #         target_action=cam_node_2,
        #         on_start=[
        #             close_cam_2
        #         ]
        #     )
        # ),

        # RegisterEventHandler(
        #     OnProcessStart(
        #         target_action=cam_node_3,
        #         on_start=[
        #             close_cam_3
        #         ]
        #     )
        # ),
 
        
        # after all camera is launch and temporarily closed, launch the remaining nodes
        cam_node_0,
        aprilTag,
        arg_container,
        image_processing_container,
        load_composable_nodes,
        converter,
        #poller,
        ekf_odom,
        ekf_pose,
        

        # give some time for localisation to stabilize before robot started to run
        TimerAction(
            period=10.0,
            actions=[robot],
        ),

        # RegisterEventHandler(
        #     OnExecutionComplete(
        #         target_action=(close_cam_0 and close_cam_1 and close_cam_2 and close_cam_3),
        #         on_completion=[
        #             aprilTag,
        #             arg_container,
        #             image_processing_container,
        #             load_composable_nodes,
        #             converter,
        #             poller,
        #             ekf_odom,
        #             ekf_pose,
                
        #             # give some time for localisation to stabilize before robot started to run
        #             TimerAction(
        #                 period=10.0,
        #                 actions=[robot],
        #             )
        #         ]
        #     )
        # ),
    ])