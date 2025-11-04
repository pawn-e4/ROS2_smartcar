from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
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

    params_substitutions = {
        'use_sim_time': 'True',
        'yaml_filename': LaunchConfiguration('map')
    }

    configured_params = RewrittenYaml(
        source_file=nav2_config,
        root_key='',
        param_rewrites=params_substitutions,
        convert_types=True
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'localization.launch.py')
        )
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
            'params_file': configured_params,
            'slam': 'False',
            'autostart': 'True'   # ✅ This makes lifecycle nodes activate automatically
        }.items(),
    )

    return LaunchDescription([
        declare_map_arg,
        localization_launch,
        bringup_launch
    ])
