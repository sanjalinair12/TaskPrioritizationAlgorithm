# robot_allocator/allocator_node.py
from ament_index_python.packages import get_package_share_directory
import os
import rclpy
from rclpy.node import Node
import numpy as np
import pandas as pd
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState



class AllocatorNode(Node):
    def __init__(self):
        super().__init__('allocator_node')
        self.get_logger().info('🚀 Robot Allocator Node Started')

        # Dynamically get the absolute path to the package's share directory
        package_share = get_package_share_directory('robot_allocator')
        data_dir = os.path.join(package_share, 'data')

        # Load input data (ensure .xlsx or .csv files exist under robot_allocator/data/)
        try:
            self.df_cc = pd.read_excel(os.path.join(data_dir, 'criteria_comparison.xlsx'), index_col=0)
            self.df_tasks = pd.read_excel(os.path.join(data_dir, 'tasks.xlsx'), index_col=0)
            self.df_robots = pd.read_excel(os.path.join(data_dir, 'robots.xlsx'), index_col=0)
            self.get_logger().info('📊 Data successfully loaded from package data directory.')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load Excel data: {e}')
            raise e

        # Run allocation once on startup
        self.allocate()

    # ----------------------------------------------------------------------
    # TOPSIS distance computation
    # ----------------------------------------------------------------------
    def topsis_distances(self, decision_matrix_norm, criteria_weights):
        """Compute TOPSIS distances and ideal points."""
        # Weighted normalized decision matrix
        v = decision_matrix_norm * criteria_weights

        # Positive and negative ideals
        ideal_plus = np.max(v, axis=0)
        ideal_minus = np.min(v, axis=0)

        # Euclidean distances
        d_plus = np.sqrt(np.sum((v - ideal_plus) ** 2, axis=1))
        d_minus = np.sqrt(np.sum((v - ideal_minus) ** 2, axis=1))

        return d_plus, d_minus, v, ideal_plus, ideal_minus

    # ----------------------------------------------------------------------
    # Gazebo teleportation service call
    # ----------------------------------------------------------------------
    def call_set_state(self, model_name, x, y):
        """Teleport robot to assigned task position using Gazebo service."""
        client = self.create_client(SetEntityState, '/set_entity_state')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /set_entity_state service...')

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = model_name
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"✅ Moved {model_name} to ({x}, {y})")
        else:
            self.get_logger().error(f"❌ Failed to move {model_name}")

    # ----------------------------------------------------------------------
    # Allocation logic (your main section)
    # ----------------------------------------------------------------------
    def allocate(self):
        df_cc = self.df_cc
        df_tasks = self.df_tasks
        df_robots = self.df_robots

        # Normalize criteria comparison matrix (AHP)
        criteria_matrix = df_cc.to_numpy()
        criteria_weights = np.mean(criteria_matrix, axis=1)
        criteria_matrix_norm = criteria_matrix / criteria_weights

        # Compute criteria weights using AHP
        criteria_weights = np.mean(criteria_matrix_norm, axis=1)

        # Normalize decision matrix (TOPSIS)
        decision_matrix = df_tasks.to_numpy()
        decision_matrix_norm = decision_matrix / np.sum(decision_matrix, axis=0)

        # Compute TOPSIS distances
        d_plus, d_minus, v, ideal_plus, ideal_minus = self.topsis_distances(
            decision_matrix_norm, criteria_weights
        )

        # 🔹 Replace VIKOR parameters s, r with TOPSIS-based dynamic ones
        s_i = d_plus / (d_plus + d_minus)
        r_i = d_minus / (d_plus + d_minus)

        # 🔹 Compute Q using adaptive hybrid formulation
        q = s_i * (d_plus - d_minus) / (d_plus + d_minus) + r_i * (
            (np.min(v, axis=1) - ideal_minus) / (np.max(v, axis=1) - ideal_minus)
        )

        # Assign tasks to robots based on Q values
        assignments = np.argmax(q)
        robot_names = df_robots['Robot'].values
        task_names = df_tasks.index.values
        robot_name = robot_names[assignments]
        task_name = task_names[assignments]

        # Get coordinates
        robot_x = df_robots['X'].values
        robot_y = df_robots['Y'].values
        task_x = df_tasks['X'].values
        task_y = df_tasks['Y'].values

        robot_idx = np.where(robot_names == robot_name)[0][0]
        task_idx = np.where(task_names == task_name)[0][0]

        # Move robot to task
        self.call_set_state(robot_name, task_x[task_idx], task_y[task_idx])

        # Logging
        self.get_logger().info(f"🤖 Assigned {robot_name} → Task {task_name}")
        self.get_logger().info(f"Q values: {q}")
        self.get_logger().info(f"Ideal+: {ideal_plus}")
        self.get_logger().info(f"Ideal-: {ideal_minus}")

# ----------------------------------------------------------------------
# ROS 2 entry point
# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = AllocatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
