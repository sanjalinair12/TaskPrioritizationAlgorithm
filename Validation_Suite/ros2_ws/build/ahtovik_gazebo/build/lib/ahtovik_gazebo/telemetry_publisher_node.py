#!/usr/bin/env python3
"""Parameterized telemetry publisher for AhToVik validation."""
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotTelemetryPublisher(Node):
    def __init__(self):
        super().__init__('robot_telemetry_publisher')
        self.noise_level = float(self.declare_parameter('noise_level', 0.0).value)
        self.seed = int(self.declare_parameter('seed', 42).value)
        self.num_drones = int(self.declare_parameter('num_drones', 3).value)
        self.num_ground = int(self.declare_parameter('num_ground', 4).value)
        self.num_creepers = int(self.declare_parameter('num_creepers', 3).value)
        self.telemetry_period = float(self.declare_parameter('telemetry_period', 1.0).value)
        random.seed(self.seed)
        self.pub = self.create_publisher(String, '/ahtovik/robot_telemetry', 10)
        self.timer = self.create_timer(self.telemetry_period, self.publish_telemetry)
        self.robots = []
        for i in range(self.num_drones):
            self.robots.append({'id': f'uav_{i}', 'type': 'Drone', 'battery': random.uniform(85, 100), 'health': 1.0, 'payload': random.uniform(1, 3), 'speed': random.uniform(7, 10), 'distance': random.uniform(5, 25), 'available': 1})
        for i in range(self.num_ground):
            self.robots.append({'id': f'ugv_{i}', 'type': 'Ground Robot', 'battery': random.uniform(75, 100), 'health': 1.0, 'payload': random.uniform(6, 10), 'speed': random.uniform(2, 5), 'distance': random.uniform(5, 25), 'available': 1})
        for i in range(self.num_creepers):
            self.robots.append({'id': f'creeper_{i}', 'type': 'Creeper Robot', 'battery': random.uniform(70, 95), 'health': 1.0, 'payload': random.uniform(1, 4), 'speed': random.uniform(1, 3), 'distance': random.uniform(2, 15), 'available': 1})
        self.get_logger().info(
            f'Telemetry publisher started: drones={self.num_drones}, ground={self.num_ground}, '
            f'creepers={self.num_creepers}, noise={self.noise_level}, seed={self.seed}'
        )

    def noisy(self, value, lower=None, upper=None):
        if self.noise_level > 0:
            value = value * (1.0 + random.gauss(0.0, self.noise_level))
        if lower is not None:
            value = max(lower, value)
        if upper is not None:
            value = min(upper, value)
        return value

    def update_robot(self, r):
        if r['available'] == 0:
            return
        r['battery'] = max(0.0, r['battery'] - random.uniform(0.05, 0.4))
        r['health'] = max(0.0, min(1.0, r['health'] + random.uniform(-0.015, 0.005)))
        r['distance'] = max(0.1, r['distance'] + random.uniform(-1.0, 1.0))
        r['speed'] = max(0.1, r['speed'] + random.uniform(-0.2, 0.2))
        if r['battery'] < 5 or r['health'] < 0.2:
            r['available'] = 0

    def publish_telemetry(self):
        for r in self.robots:
            self.update_robot(r)
            msg = String()
            battery = self.noisy(r['battery'], 0.0, 100.0)
            health = self.noisy(r['health'], 0.0, 1.0)
            payload = self.noisy(r['payload'], 0.0, 20.0)
            speed = self.noisy(r['speed'], 0.0, 20.0)
            distance = self.noisy(r['distance'], 0.1, 200.0)
            msg.data = f"{r['id']},{r['type']},{battery:.2f},{health:.2f},{payload:.2f},{speed:.2f},{distance:.2f},{r['available']}"
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotTelemetryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
