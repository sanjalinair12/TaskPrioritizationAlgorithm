#!/usr/bin/env python3
"""Configurable failure injector for ROS2 validation."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FailureInjector(Node):
    def __init__(self):
        super().__init__('failure_injector')
        self.fail_robot = str(self.declare_parameter('fail_robot', 'uav_0').value)
        self.fail_time = float(self.declare_parameter('fail_time', 12.0).value)
        self.enabled = bool(self.declare_parameter('enabled', True).value)
        self.pub = self.create_publisher(String, '/ahtovik/failure_events', 10)
        self.timer = self.create_timer(self.fail_time, self.inject_once)
        self.done = False
        self.get_logger().info(f'Failure injector enabled={self.enabled}, robot={self.fail_robot}, t={self.fail_time}s')

    def inject_once(self):
        if self.done or not self.enabled:
            return
        msg = String(); msg.data = self.fail_robot
        self.pub.publish(msg)
        self.get_logger().warn(f'Injected failure for {self.fail_robot}')
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = FailureInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
