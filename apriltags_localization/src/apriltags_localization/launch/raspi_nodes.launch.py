from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessStart, OnExecutionComplete
from launch.substitutions import FindExecutable     
from launch_ros.substitutions import FindPackageShare
import os           


ROBOT_ID = '0'
pkg_share = FindPackageShare(package='apriltags_localization').find('apriltags_localization')

robot_localization_file_path = os.path.join(pkg_share, 'config/ekf_odom.yaml') 
ekf_tags_file_path = os.path.join(pkg_share, 'config/ekf.yaml') 

def generate_launch_description():

        # launch the cameras
        cam_node_0 =  Node(
            package = 'usb_cam',
            executable = 'usb_cam_node_exe',
            name = 'robot_'+ROBOT_ID+'_cam_0',
            
            parameters=[

                {'video_device': '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0'},
                {'camera_name': 'robot_'+ROBOT_ID+'/cam_0'},
                {'camera_info_url': 'file:///home/ubuntu/github_repo/networked_robotics/calibration/cam_0.yaml'},
                {'frame_id': 'robot_'+ROBOT_ID+"/cam_0"},
                {'pixel_format':'yuyv'},
                {'framerate': 5.0},
                {'image_height': 480},
                {'image_width':640},
                {'io_method': 'userptr'}
              
                
            ],
            remappings=[
            ('image_raw', 'robot_'+ROBOT_ID+'/image_raw'),
            ('set_capture','robot_'+ROBOT_ID+'/cam_0/set_capture'),
            ('camera_info', 'robot_'+ROBOT_ID+'/camera_info'),
            ('rosout', 'robot_'+ROBOT_ID+'/rosout/cam')

            
           
          

            ]
        )

        cam_node_1 =  Node(
            package = 'usb_cam',
            executable = 'usb_cam_node_exe',
            name = 'robot_'+ROBOT_ID+'_cam_1',
            
            parameters=[

                {'video_device': '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-video-index0'},
                {'camera_name': 'robot_'+ROBOT_ID+'/cam_1'},
                {'camera_info_url': 'file:///home/ubuntu/github_repo/networked_robotics/calibration/cam_1.yaml'},
                {'frame_id': 'robot_'+ROBOT_ID+"/cam_1"},
                {'framerate': 5.0},
                {'image_height': 480},
                {'image_width':640},
                {'io_method': 'userptr'}
            ],
            remappings=[
            ('image_raw', 'robot_'+ROBOT_ID+'/image_raw'),
            ('set_capture','robot_'+ROBOT_ID+'/cam_1/set_capture'),
            ('camera_info', 'robot_'+ROBOT_ID+'/camera_info'),
            ('rosout', 'robot_'+ROBOT_ID+'/rosout/cam')


            ]
        )

        cam_node_2 =  Node(
            package = 'usb_cam',
            executable = 'usb_cam_node_exe',
            name = 'robot_'+ROBOT_ID+'_cam_2',
            
            parameters=[

                {'video_device': '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-video-index0'},
                {'camera_name': 'robot_'+ROBOT_ID+'/cam_2'},
                {'camera_info_url': 'file:///home/ubuntu/github_repo/networked_robotics/calibration/cam_2.yaml'},
                {'frame_id': 'robot_'+ROBOT_ID+"/cam_2"},
                {'framerate': 5.0},
                {'image_height': 480},
                {'image_width':640},
                {'io_method': 'userptr'}

            ],
            remappings=[
            ('image_raw', 'robot_'+ROBOT_ID+'/image_raw'),
            ('set_capture','robot_'+ROBOT_ID+'/cam_2/set_capture'),
            ('camera_info', 'robot_'+ROBOT_ID+'/camera_info'),
            ('rosout', 'robot_'+ROBOT_ID+'/rosout/cam')


            ]
        )

        cam_node_3 =  Node(
            package = 'usb_cam',
            executable = 'usb_cam_node_exe',
            name = 'robot_'+ROBOT_ID+'_cam_3',
            
            parameters=[

                {'video_device': '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0'},
                {'camera_name': 'robot_'+ROBOT_ID+'/cam_3'},
                {'camera_info_url': 'file:///home/ubuntu/github_repo/networked_robotics/calibration/cam_3.yaml'},
                {'frame_id': 'robot_'+ROBOT_ID+"/cam_3"},
                {'framerate': 5.0},
                {'image_height': 480},
                {'image_width':640},
                {'io_method': 'userptr'}
            ],
            remappings=[
            ('image_raw', 'robot_'+ROBOT_ID+'/image_raw'),
            ('set_capture','robot_'+ROBOT_ID+'/cam_3/set_capture'),
            ('camera_info', 'robot_'+ROBOT_ID+'/camera_info'),
            ('rosout', 'robot_'+ROBOT_ID+'/rosout/cam')
 

            ]
        )

        # launch the apriltag nodes
        aprilTag =  Node(
            package = 'apriltag_ros',
            executable = 'apriltag_node',
            name = 'robot_'+ROBOT_ID+'_aprilTag',
            
            parameters=[
                '/home/ubuntu/github_repo/networked_robotics/apriltags_localization/src/apriltags_localization/config/aprilTag.yaml',
                
            ],

            remappings=[
                ('image_rect', 'robot_'+ROBOT_ID+'/image_raw'),
                ('tf', 'robot_'+ROBOT_ID+'/tf_detections'),
                ('camera_info', 'robot_'+ROBOT_ID+'/camera_info'),
                ('detections', 'robot_'+ROBOT_ID+'/detections')
            ]
            
        )

        

        
        
        ## event handlers

        ## close the camera once it is started to prevent bandwidth issues
        close_cam_0 = ExecuteProcess(
                cmd=[[
                FindExecutable(name='ros2'),
                ' service call ',
                'robot_'+ROBOT_ID+'/cam_0/set_capture',
                ' std_srvs/srv/SetBool',
                ' "{data : False}" '

                ]],
                shell=True
        )

        close_cam_1 = ExecuteProcess(
                cmd=[[
                FindExecutable(name='ros2'),
                ' service call ',
                'robot_'+ROBOT_ID+'/cam_1/set_capture ',
                ' std_srvs/srv/SetBool ',
                ' "{data : False}" '

                ]],
                shell=True
        )

        close_cam_2 = ExecuteProcess(
                cmd=[[
                FindExecutable(name='ros2'),
                ' service call ',
                'robot_'+ROBOT_ID+'/cam_2/set_capture ',
                ' std_srvs/srv/SetBool ',
                '  "{data : False}" '

                ]],
                shell=True
        )

        close_cam_3 = ExecuteProcess(
                cmd=[[
                FindExecutable(name='ros2'),
                ' service call ',
                'robot_'+ROBOT_ID+'/cam_3/set_capture ',
                ' std_srvs/srv/SetBool ',
                '  "{data : False}" '

                ]],
                shell=True
        )

        # launch the poller node
        poller = Node(
            package = 'apriltags_localization',
            executable = 'poller',
            name = 'robot_'+ROBOT_ID+'_poller',
            parameters=[
                {'robot_id': ROBOT_ID}
            ]

        )

        # the robot PID controller node
        robot = Node(
            package = 'control',
            executable = 'robot',
            name = 'robot_'+ROBOT_ID,
            parameters=[
                {'robot_id': ROBOT_ID}
            ]

        )

        # converter node to invert the transform of cam->tag
        converter = Node(
            package='apriltags_localization',
            executable='converter',
            name= 'robot_'+ROBOT_ID+'_converter',
            parameters=[
                {'robot_id': ROBOT_ID}
            ]
        )

        ## node for odometry
        ekf_odom = Node(

            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            parameters=[robot_localization_file_path
            ],
            remappings=[
            ('/odometry/filtered', '/robot_0/filtered_odom'),
            ('diagnostics', 'diagnostics_odom'),
            
            ]
            
            )

        ## node for pose (tag detection)    
        
        ekf_pose = Node(

            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_pose',
            parameters=[ekf_tags_file_path
            ],
            remappings=[
            ('/odometry/filtered', '/robot_0/filtered_pose'),
            ('diagnostics', 'diagnostics_pose'),
            
            ]
            )

        commander =  Node(

            package='control',
            executable='commander'
        )




        
                

        return LaunchDescription([

                ## static frame for base_link to cameras
        
                

                # launch the remainings node
                cam_node_0,

                
                RegisterEventHandler(
                        OnProcessStart(
                                target_action=cam_node_0,
                                on_start=[
                                
                                TimerAction(
                                    period=3.0,
                                    actions=[cam_node_1],
                                ),
                                TimerAction(
                                    period=3.0,
                                    actions=[cam_node_2],
                                ),
                                TimerAction(
                                    period=3.0,
                                    actions=[cam_node_3],
                                )
 
                                ]
                        )
                        ),
      

                RegisterEventHandler(
                        OnProcessStart(
                                target_action=cam_node_0,
                                on_start=[
                                
                                close_cam_0
 
                                ]
                        )
                        ),
                
                RegisterEventHandler(
                        OnProcessStart(
                                target_action=cam_node_1,
                                on_start=[
                                
                                close_cam_1

                                ]
                        )
                        ),


                RegisterEventHandler(
                        OnProcessStart(
                                target_action=cam_node_2,
                                on_start=[
                                
                                close_cam_2

                                ]
                        )
                        ),

                RegisterEventHandler(
                        OnProcessStart(
                                target_action=cam_node_3,
                                on_start=[
                                
                                close_cam_3
                                ]
                        )
                        ),
               
               RegisterEventHandler(
                        OnExecutionComplete(
                                target_action=(close_cam_0 and close_cam_1 and close_cam_2 and close_cam_3),
                                on_completion=[
                                
                                aprilTag,
                                converter,
                                poller,
                                ekf_pose,
                                ekf_odom,
                                

                                TimerAction(
                                    period=10.0,
                                    actions=[robot],
                                )
                                
                                ]
                        )
                        ),





                
        ])