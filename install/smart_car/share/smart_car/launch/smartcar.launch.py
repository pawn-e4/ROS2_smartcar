from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Path to the URDF file
    urdf_file = PathJoinSubstitution([
        FindPackageShare("smart_car"),
        "urdf",
        "smartcar.urdf"
    ])

    # Load URDF contents properly as string
    robot_description = ParameterValue(
        Command(["cat ", urdf_file]),
        value_type=str
    )

    # Path to the RViz configuration file
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("smart_car"),
        "rviz",
        "smartcar.rviz"
    ])

    return LaunchDescription([
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen"
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file]
        )
    ])
