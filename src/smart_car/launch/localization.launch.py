from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share  = get_package_share_directory('smart_car')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf_local.yaml')

    # Wheel odometry: publishes odom -> base_footprint TF and /smart_car/wheel/odom topic
    wheel_odometry = Node(
        package='smart_car',
        executable='wheel_odometry',
        name='wheel_odometry',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ],
        emulate_tty=True,
        # If your plugin publishes vehicle status on a different topic, uncomment & adjust:
        # remappings=[('/smart_car/vehicle_status', '/vehicle_status')],
    )

    # Your joint state publisher node (from smart_car/scripts/joint_state_publisher.py)
    joint_state_pub = Node(
        package='smart_car',
        executable='joint_state_publisher',
        name='joint_state_tf',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # EKF: fuses /odom + /imu_data and publishes /odometry/filtered (TF handled by wheel odom)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}]
        # Do NOT remap odometry/filtered -> /odom; wheel odom already covers TF + odom topic
    )

    return LaunchDescription([
        wheel_odometry,
        joint_state_pub,
        ekf_node
    ])
