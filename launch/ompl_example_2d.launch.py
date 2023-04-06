import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():

    backtrace = LaunchConfiguration('backtrace', default='False')

    map_path = os.path.join(get_package_share_directory('example_maps'), 'maps', 'map_home.yaml')

    map_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('example_maps'), 'launch', 'map_launch.py')
        ),
        launch_arguments={'map': map_path}.items()
    )


    rviz_config_dir = os.path.join(
        get_package_share_directory('example_maps'), 'rviz', 'rviz_cfg.rviz')

    rviz_cmd = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            output='screen')

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(map_server_cmd)
    ld.add_action(rviz_cmd)

    return ld
