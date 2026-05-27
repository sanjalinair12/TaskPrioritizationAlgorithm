#!/usr/bin/env python3
"""CSV logger for AhToVik validation experiments."""
import csv
import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ExperimentLogger(Node):
    def __init__(self):
        super().__init__('ahtovik_experiment_logger')
        default_dir = str(Path.home() / 'ahtovik_validation_results')
        self.output_dir = Path(str(self.declare_parameter('output_dir', default_dir).value)).expanduser()
        self.experiment_id = str(self.declare_parameter('experiment_id', 'manual').value)
        self.run_id = int(self.declare_parameter('run_id', 0).value)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.metrics_path = self.output_dir / f'{self.experiment_id}_run_{self.run_id}_metrics.csv'
        self.events_path = self.output_dir / f'{self.experiment_id}_run_{self.run_id}_events.csv'
        self.metrics_file = open(self.metrics_path, 'w', newline='')
        self.events_file = open(self.events_path, 'w', newline='')
        self.metrics_writer = None
        self.events_writer = csv.DictWriter(self.events_file, fieldnames=['timestamp', 'event_type', 'payload'])
        self.events_writer.writeheader()
        self.create_subscription(String, '/ahtovik/metrics', self.metrics_callback, 10)
        self.create_subscription(String, '/ahtovik/failure_events', self.failure_callback, 10)
        self.create_subscription(String, '/ahtovik/task_assignments', self.assignment_callback, 10)
        self.get_logger().info(f'Logging metrics to {self.metrics_path}')

    def write_event(self, event_type, payload):
        self.events_writer.writerow({'timestamp': time.time(), 'event_type': event_type, 'payload': payload})
        self.events_file.flush()

    def failure_callback(self, msg):
        self.write_event('failure', msg.data)

    def assignment_callback(self, msg):
        self.write_event('assignment_text', msg.data.replace('\n', ' | '))

    def metrics_callback(self, msg):
        try:
            data = json.loads(msg.data)
            # Flatten large fields to JSON strings so the CSV remains easy to parse.
            flat = dict(data)
            for key in ['assignments', 'q_snapshot', 'recovery_times_s']:
                flat[key] = json.dumps(flat.get(key, {}))
            if self.metrics_writer is None:
                self.metrics_writer = csv.DictWriter(self.metrics_file, fieldnames=list(flat.keys()))
                self.metrics_writer.writeheader()
            self.metrics_writer.writerow(flat)
            self.metrics_file.flush()
        except Exception as exc:
            self.get_logger().warn(f'Could not log metrics: {exc} | {msg.data}')

    def destroy_node(self):
        try:
            self.metrics_file.close()
            self.events_file.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
