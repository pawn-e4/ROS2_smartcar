from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='car_gazebo_plugin',
            executable='car_gazebo_plugin_node',  # replace with your plugin executable
            name='car_gazebo_plugin',
            output='screen'
        )
    ])
