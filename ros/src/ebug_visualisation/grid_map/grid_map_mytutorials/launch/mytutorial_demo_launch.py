import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Find the grid_map_demos package share directory
    grid_map_demos_dir = get_package_share_directory('grid_map_mytutorials')

    # Declare launch configuration variables that can access the launch arguments values
    visualization_config_file = LaunchConfiguration('visualization_config')
    rviz_config_file = LaunchConfiguration('rviz_config')

    # Declare launch arguments
    declare_visualization_config_file_cmd = DeclareLaunchArgument(
        'visualization_config',
        default_value=os.path.join(
            grid_map_demos_dir, 'config', 'mytutorial_demo.yaml'),
        description='Full path to the Gridmap visualization config file to use')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            grid_map_demos_dir, 'rviz', 'grid_map_mytutorial.rviz'),
        description='Full path to the RVIZ config file to use')

    # Declare node actions
    simple_demo_node = Node(
        package='grid_map_mytutorials',
        executable='physical_robot_demo',
        name='physical_robot_demo',
        output='screen'
    )

    echo_pose_node = Node(
        package='grid_map_mytutorials',
        executable='echo_pose',
        name='echo_pose',
        parameters=[
            {'target_frame': LaunchConfiguration('target_frame')}
        ]
    )

    robot_pose_grid_node = Node(
        package='grid_map_mytutorials',
        executable='robot_pose_grid',
        name='pose_listener',
        parameters=[
            {'target_frame': LaunchConfiguration('target_frame')}
        ]
    )

    grid_map_visualization_node = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[visualization_config_file]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    tf_listener = Node(
        package='grid_map_mytutorials',
        executable='tf_listener'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add launch arguments to the launch description
    ld.add_action(declare_visualization_config_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add node actions to the launch description
    ld.add_action(simple_demo_node)
    ld.add_action(grid_map_visualization_node)
    ld.add_action(rviz2_node)
    # ld.add_action(echo_pose_node)
    ld.add_action(tf_listener)

    return ld
