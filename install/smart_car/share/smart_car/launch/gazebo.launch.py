from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('smart_car')
    gazebo_pkg_share = get_package_share_directory('gazebo_ros')

    # ---- Paths
    world_file = os.path.join(pkg_share, 'world', 'smalltown.world')
    urdf_file  = os.path.join(pkg_share, 'urdf', 'smartcar.urdf')
    rviz_file  = os.path.join(pkg_share, 'rviz', 'smartcar.rviz')

    # ---- Args
    world_arg = DeclareLaunchArgument('world', default_value=world_file)
    spawn_arg = DeclareLaunchArgument('spawn_robot', default_value='true')  

    # ---- Gazebo (server+client)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # ---- Shared params
    robot_description = ParameterValue(Command(['cat ', urdf_file]), value_type=str)
    set_sim_time        = SetParameter(name='use_sim_time', value=True)
    set_robot_desc_param= SetParameter(name='robot_description', value=robot_description)

    # ---- One robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
    )

    # ---- Conditionally spawn the robot into Gazebo
    def maybe_spawn_robot(context, *args, **kwargs):
        spawn_flag = LaunchConfiguration('spawn_robot').perform(context).lower()
        if spawn_flag in ('true', '1', 'yes', 'on'):
            return [Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'smart_car', '-topic', 'robot_description'],
                output='screen'
            )]
        return []

    spawn_entity_group = OpaqueFunction(function=maybe_spawn_robot)

    # ---- RViz start
    rviz = TimerAction(
        period=3.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': True}],
        )]
    )

    return LaunchDescription([
        set_sim_time,
        set_robot_desc_param,
        world_arg,
        spawn_arg,
        LogInfo(msg=['Loading world: ', LaunchConfiguration('world')]),
        gazebo,
        robot_state_publisher,
        spawn_entity_group,
        rviz
    ])
