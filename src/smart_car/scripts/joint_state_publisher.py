#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from smartcar_msgs.msg import Status
import math


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        # --- Use Gazebo's simulated time if available ---
    
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])

        # --- Subscribe to vehicle status topic ---
        self.create_subscription(Status, '/smart_car/vehicle_status', self.status_callback, 10)

        # --- Publisher for joint states ---
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # --- Timer for publishing joint states ---
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_joint_states)

        # --- Variables ---
        self.engine_speed_rpm = 0.0
        self.steering_angle = 0.0
        self.position = 0.0  # wheel rotation
        self.wheel_radius = 0.032  # meters (matches URDF)

        self.get_logger().info('Joint state publisher initialized using Gazebo clock.')

    def status_callback(self, msg):
        """Update from vehicle status message."""
        self.engine_speed_rpm = msg.engine_speed_rpm
        self.steering_angle = msg.steering_angle_rad

    def publish_joint_states(self):
        """Publish wheel and steering joint angles."""
        # Convert engine speed (RPM) → radians per second
        wheel_rps = self.engine_speed_rpm / 60.0
        angular_velocity = wheel_rps * 2.0 * math.pi

        # Integrate rotation over time
        self.position += angular_velocity * self.timer_period

        # Prepare message
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [
            'front_left_wheel_steer_joint',
            'front_right_wheel_steer_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'back_left_wheel_joint',
            'back_right_wheel_joint'
        ]
        js.position = [
            self.steering_angle,  # front_left_wheel_steer_joint
            self.steering_angle,  # front_right_wheel_steer_joint
            self.position,        # front_left_wheel_joint
            self.position,        # front_right_wheel_joint
            self.position,        # back_left_wheel_joint
            self.position         # back_right_wheel_joint
        ]
        js.velocity = [
            0.0, 0.0,
            angular_velocity, angular_velocity,
            angular_velocity, angular_velocity
        ]

        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

