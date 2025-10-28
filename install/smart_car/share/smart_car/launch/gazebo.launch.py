from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('smart_car')
    gazebo_pkg_share = get_package_share_directory('gazebo_ros')

    # ALWAYS use pkg_share so the install space works
    world_file = os.path.join(pkg_share, 'world', 'smalltown.world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Absolute path to the world file'
    )
    log_world = LogInfo(msg=['Loading world: ', world_file])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Load URDF text
    urdf_file = os.path.join(pkg_share, 'urdf', 'smartcar.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Global params first, so every node sees /clock + robot_description
    set_sim_time = SetParameter(name='use_sim_time', value=True)
    set_robot_description = SetParameter(name='robot_description', value=robot_desc)

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, {'publish_default_positions': True}],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}, {'use_sim_time': True}],
    )

    # ← wheel odom as a proper ROS2 node with respawn + tty to surface errors
    wheel_odometry = Node(
        package='smart_car',
        executable='wheel_odometry',
        name='wheel_odometry',
        output='screen',
        parameters=[{'use_sim_time': True}],
        emulate_tty=True,
        respawn=True,
        respawn_delay=1.0,
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'smart_car', '-topic', 'robot_description'],
        output='screen'
    )

    rviz_config_file = os.path.join(pkg_share, 'rviz', 'smartcar.rviz')
    rviz = TimerAction(
        period=3.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}],
        )]
    )

    return LaunchDescription([
        set_sim_time,
        set_robot_description,
        world_arg,
        log_world,
        gazebo,
        joint_state_publisher,
        robot_state_publisher,
        wheel_odometry,
        spawn_robot,
        rviz,
    ])

