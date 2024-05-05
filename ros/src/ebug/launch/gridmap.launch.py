import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    PKG_SHARE = FindPackageShare(package='ebug').find('ebug')

    # Configuration files
    GRIDMAP_CONFIG = os.path.join(PKG_SHARE, 'config/gridmap.yaml')
    RVIZ_CONFIG = os.path.join(PKG_SHARE, 'rviz/gridmap.rviz')

    return LaunchDescription([
        Node(
            package='ebug_base',
            executable='gridmap_controller',
            name='GridmapController',
            output='screen'
        ),
        Node(
            package='grid_map_visualization',
            executable='grid_map_visualization',
            name='grid_map_visualization',
            output='screen',
            parameters=[GRIDMAP_CONFIG]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', RVIZ_CONFIG]
        )
    ])

