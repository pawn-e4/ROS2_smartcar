from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('smart_car')
    nav2_config = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg_share, 'nav2_map', 'smalltown_world.yaml')

    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load'
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ]),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'True',
            'params_file': nav2_config,
            'slam': 'False'
        }.items(),
    )

    return LaunchDescription([
        declare_map_arg,
        bringup_launch
    ])
