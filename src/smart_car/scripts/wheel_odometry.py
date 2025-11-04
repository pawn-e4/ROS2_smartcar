#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from smartcar_msgs.msg import Status
import tf_transformations


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])

        # Correct parameters
        self.declare_parameter('wheel_radius', 0.032)   # meters
        self.declare_parameter('wheel_base', 0.257)     # meters (distance between front & rear axle)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value

        # Publishers
        self.publisher_ = self.create_publisher(Odometry, '/smart_car/wheel/odom', 10)
        self.br = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(Status, '/smart_car/vehicle_status', self.status_callback, 10)

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.steering_angle = 0.0
        self.last_time = self.get_clock().now()

        # Timer to publish odometry
        self.create_timer(0.1, self.publish_odom)

        self.get_logger().info('✅ Wheel odometry node initialized (corrected parameters + no duplicate TF).')

    def status_callback(self, msg):
        # Convert engine RPM to linear velocity
        wheel_rps = msg.engine_speed_rpm / 60.0
        self.v = (2.0 * math.pi * self.wheel_radius) * wheel_rps
        self.steering_angle = msg.steering_angle_rad

    def publish_odom(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Bicycle model integration
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += (self.v / self.wheel_base) * math.tan(self.steering_angle) * dt

        # Quaternion from yaw
        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)

        # TF: odom → base_footprint
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = (self.v / self.wheel_base) * math.tan(self.steering_angle)

        self.publisher_.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
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
