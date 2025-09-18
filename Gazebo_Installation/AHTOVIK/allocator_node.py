import rclpy
from rclpy.node import Node
import numpy as np
import pandas as pd
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class AllocatorNode(Node):
    def __init__(self):
        super().__init__('allocator_node')

        # Service client for teleportation in Gazebo
        self.cli = self.create_client(SetModelState, '/gazebo/set_model_state')
        self.get_logger().info('Waiting for gazebo set_model_state service...')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_model_state service...')
        self.get_logger().info('Service available.')

    def call_set_state(self, model_name, x, y, z=0.0):
        """Teleport a robot to new coordinates in Gazebo"""
        req = SetModelState.Request()
        state = ModelState()
        state.model_name = model_name
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = float(z)
        state.pose.orientation.w = 1.0  # neutral orientation
        req.model_state = state

        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is None:
            self.get_logger().warn(f"Failed to set state for {model_name}")
        else:
            self.get_logger().info(f"Teleported {model_name} to ({x:.2f},{y:.2f},{z:.2f})")

    def compute_and_assign(self, excel_path: str):
        """Run AHTOVIK allocation and move robots in Gazebo"""

        # Load data from Excel
        try:
            df_cc = pd.read_excel(excel_path, sheet_name='Criteria Comparison', index_col=0)
            df_robots = pd.read_excel(excel_path, sheet_name='Robots')
            df_tasks = pd.read_excel(excel_path, sheet_name='Tasks', index_col=0)
        except Exception as e:
            self.get_logger().error(f"Error reading Excel file: {e}")
            return

        criteria_comparison_matrix = df_cc.values

        # Build robot list
        robots = []
        for _, row in df_robots.iterrows():
            robots.append({'ID': str(row['ID']), 'Type': row['Type'],
                           'Values': row.iloc[2:].values.astype(float)})

        # Build task list
        tasks = []
        for idx, row in df_tasks.iterrows():
            tasks.append({'Type': idx, 'Scores': row.values.astype(float)})

        # --- AHP Criteria Weights ---
        eigenvalues, eigenvectors = np.linalg.eig(criteria_comparison_matrix)
        max_eigenvalue_index = np.argmax(eigenvalues)
        criteria_weights = np.abs(eigenvectors[:, max_eigenvalue_index])
        criteria_weights = criteria_weights / np.sum(criteria_weights)

        # --- Task Priorities ---
        task_priorities = {t['Type']: np.dot(t['Scores'], criteria_weights) for t in tasks}
        sorted_tasks = sorted(task_priorities.items(), key=lambda x: x[1], reverse=True)

        self.get_logger().info("Task Priorities:")
        for task, priority in sorted_tasks:
            self.get_logger().info(f"{task}: {priority:.4f}")

        # --- ROBOT PERFORMANCE EVALUATION---
        decision_matrix = np.array([robot['Values'] for robot in robots])
        weighted_matrix = decision_matrix * criteria_weights
        ideal_best = weighted_matrix.max(axis=0)
        ideal_worst = weighted_matrix.min(axis=0)
        distance_best = np.sqrt(np.sum((weighted_matrix - ideal_best) ** 2, axis=1))
        distance_worst = np.sqrt(np.sum((weighted_matrix - ideal_worst) ** 2, axis=1))

        # VIKOR BASED RANKING
        D_plus = distance_best
        D_minus = distance_worst
        v = 0.5
        D_plus_best, D_plus_worst = np.min(D_plus), np.max(D_plus)
        D_minus_best, D_minus_worst = np.min(D_minus), np.max(D_minus)

        if (D_plus_worst - D_plus_best) == 0 or (D_minus_worst - D_minus_best) == 0:
            Q = D_plus
        else:
            Q = v * ((D_plus - D_plus_best) / (D_plus_worst - D_plus_best)) \
              + (1-v) * ((D_minus_worst - D_minus) / (D_minus_worst - D_minus_best))

        # --- Rank robots by type ---
        ground_robots = sorted([r for r in robots if r['Type'] == 'Ground Robot'],
                               key=lambda r: Q[robots.index(r)])
        drone_robots = sorted([r for r in robots if r['Type'] == 'Drone'],
                              key=lambda r: Q[robots.index(r)])
        creeper_robots = sorted([r for r in robots if r['Type'] == 'Creeper Robot'],
                                key=lambda r: Q[robots.index(r)])

        # --- Assignment helper ---
        def get_robot_score(robot, weights):
            return np.dot(robot['Values'], weights)

        # Predefined waypoints for demo
        waypoints = {
            'Scan and Rescue': (10.0, 0.0, 5.0),
            'Lifting Remains': (-5.0, 5.0, 0.0),
            'Supply Necessities': (0.0, -8.0, 0.0),
        }

        # --- Assign robots per task ---
        for task_type, task_priority in sorted_tasks:
            self.get_logger().info(f"\n{task_type} Priority: {task_priority:.4f}")
            assigned_robots = []
            task = next(t for t in tasks if t['Type'] == task_type)
            weights = task['Scores'] / np.sum(task['Scores'])

            if task_type == 'Scan and Rescue':
                if drone_robots:
                    best_drone = max(drone_robots, key=lambda r: get_robot_score(r, weights))
                    assigned_robots.append(best_drone); drone_robots.remove(best_drone)
                for _ in range(2):
                    if ground_robots:
                        best_ground = max(ground_robots, key=lambda r: get_robot_score(r, weights))
                        assigned_robots.append(best_ground); ground_robots.remove(best_ground)

            elif task_type == 'Lifting Remains':
                for _ in range(2):
                    if ground_robots:
                        best_ground = max(ground_robots, key=lambda r: get_robot_score(r, weights))
                        assigned_robots.append(best_ground); ground_robots.remove(best_ground)

            elif task_type == 'Supply Necessities':
                if ground_robots:
                    best_ground = max(ground_robots, key=lambda r: get_robot_score(r, weights))
                    assigned_robots.append(best_ground); ground_robots.remove(best_ground)
                if creeper_robots:
                    best_creeper = max(creeper_robots, key=lambda r: get_robot_score(r, weights))
                    assigned_robots.append(best_creeper); creeper_robots.remove(best_creeper)

            if assigned_robots:
                self.get_logger().info(f"Team for {task_type}:")
                wp = waypoints.get(task_type, (0, 0, 0))
                for i, robot in enumerate(assigned_robots):
                    self.get_logger().info(f"{robot['ID']} ({robot['Type']})")
                    # Teleport robots into Gazebo
                    self.call_set_state(robot['ID'],
                                        wp[0] + i * 1.0,
                                        wp[1] + i * 1.0,
                                        wp[2])
            else:
                self.get_logger().warn(f"No robots assigned to {task_type}.")

def main(args=None):
    rclpy.init(args=args)
    node = AllocatorNode()
    excel_path = "/absolute/path/to/robot_criteria_populated.xlsx"
    node.compute_and_assign(excel_path)
    rclpy.spin_once(node, timeout_sec=0.1)  # keep alive briefly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

