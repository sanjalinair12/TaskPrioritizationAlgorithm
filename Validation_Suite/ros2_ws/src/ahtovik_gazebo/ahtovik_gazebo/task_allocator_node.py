#!/usr/bin/env python3
"""AhToVik vs CBBA Gazebo/ROS2 validation allocator.

Algorithms supported:
  - ahtovik: L2 ideal-distance compromise ranking with threshold-oriented recovery.
  - cbba: simplified Consensus-Based Bundle Algorithm baseline simulator.

The CBBA mode is intentionally implemented as a validation baseline, not as a
full network stack. It models bundle bidding, conflict resolution rounds,
consensus message count, and recovery after a failure event so that AhToVik and
CBBA can be compared under the same telemetry/noise/Gazebo scenario.
"""
import json
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class RobotState:
    robot_id: str
    robot_type: str
    battery: float = 100.0
    health: float = 1.0
    payload: float = 0.0
    speed: float = 0.0
    distance: float = 0.0
    available: bool = True
    last_q: Dict[str, float] = field(default_factory=dict)


@dataclass
class TaskSlot:
    slot_id: str
    task_name: str
    robot_type: str
    slot_index: int


class AhToVikAllocator(Node):
    def __init__(self):
        super().__init__('ahtovik_task_allocator')
        self.v = float(self.declare_parameter('v', 0.5).value)
        self.threshold = float(self.declare_parameter('threshold', 0.10).value)
        self.algorithm = str(self.declare_parameter('algorithm', 'ahtovik').value).lower()
        self.assignment_period = float(self.declare_parameter('assignment_period', 2.0).value)
        self.experiment_id = str(self.declare_parameter('experiment_id', 'manual').value)
        self.noise_level = float(self.declare_parameter('noise_level', 0.0).value)
        self.run_id = int(self.declare_parameter('run_id', 0).value)
        self.cbba_max_rounds = int(self.declare_parameter('cbba_max_rounds', 20).value)
        self.cbba_bundle_size = int(self.declare_parameter('cbba_bundle_size', 3).value)
        self.cbba_distance_penalty = float(self.declare_parameter('cbba_distance_penalty', 0.001).value)

        self.weights = np.array([0.25, 0.25, 0.20, 0.20, 0.10], dtype=float)
        self.tasks = {
            'Scan and Rescue': {'scores': np.array([0.25, 0.30, 0.05, 0.25, 0.15]), 'requirements': {'Drone': 1, 'Ground Robot': 2}},
            'Supply Necessities': {'scores': np.array([0.20, 0.20, 0.35, 0.10, 0.15]), 'requirements': {'Ground Robot': 1, 'Creeper Robot': 1}},
            'Lifting Remains': {'scores': np.array([0.15, 0.25, 0.40, 0.05, 0.15]), 'requirements': {'Ground Robot': 1, 'Creeper Robot': 1}},
        }
        self.robots: Dict[str, RobotState] = {}
        self.previous_assignments: Dict[str, List[str]] = {}
        self.failure_times: Dict[str, float] = {}
        self.recovered_failures: Dict[str, float] = {}
        self.allocation_count = 0
        self.total_reallocations = 0
        self.start_time = time.time()

        self.assignment_pub = self.create_publisher(String, '/ahtovik/task_assignments', 10)
        self.metrics_pub = self.create_publisher(String, '/ahtovik/metrics', 10)
        self.create_subscription(String, '/ahtovik/robot_telemetry', self.telemetry_callback, 10)
        self.create_subscription(String, '/ahtovik/failure_events', self.failure_callback, 10)
        self.timer = self.create_timer(self.assignment_period, self.allocate_tasks)
        self.get_logger().info(
            f'Validation allocator started: algorithm={self.algorithm}, v={self.v}, '
            f'threshold={self.threshold}, noise={self.noise_level}, run={self.run_id}'
        )

    def telemetry_callback(self, msg: String):
        try:
            data = [x.strip() for x in msg.data.split(',')]
            robot_id, robot_type = data[0], data[1]
            if robot_id not in self.robots:
                self.robots[robot_id] = RobotState(robot_id, robot_type)
            r = self.robots[robot_id]
            r.battery = float(data[2])
            r.health = float(data[3])
            r.payload = float(data[4])
            r.speed = float(data[5])
            r.distance = float(data[6])
            r.available = bool(int(data[7]))
        except Exception as e:
            self.get_logger().warn(f'Bad telemetry: {msg.data} | {e}')

    def failure_callback(self, msg: String):
        rid = msg.data.strip()
        self.failure_times[rid] = time.time()
        if rid in self.robots:
            self.robots[rid].available = False
        self.get_logger().warn(f'Failure event received: {rid}')
        self.allocate_tasks(force=True)

    def robot_vec(self, r: RobotState) -> np.ndarray:
        distance_score = 1.0 / (1.0 + max(0.0, r.distance))
        return np.array([r.battery / 100.0, r.health, r.payload / 10.0, r.speed / 10.0, distance_score], dtype=float)

    def utility(self, r: RobotState, task_scores: np.ndarray) -> float:
        return float(np.sum(self.robot_vec(r) * self.weights * task_scores))

    def score_vector(self, robots: List[RobotState], task_scores: np.ndarray) -> np.ndarray:
        if not robots:
            return np.array([])
        return np.array([self.utility(r, task_scores) for r in robots])

    def compute_ahtovik_q(self, robots: List[RobotState], task_scores: np.ndarray) -> List[float]:
        if not robots:
            return []
        matrix = np.array([self.robot_vec(r) for r in robots])
        weighted = matrix * self.weights * task_scores
        pis, nis = np.max(weighted, axis=0), np.min(weighted, axis=0)
        d_plus = np.sqrt(np.sum((weighted - pis) ** 2, axis=1))
        d_minus = np.sqrt(np.sum((weighted - nis) ** 2, axis=1))
        dp_min, dp_max = np.min(d_plus), np.max(d_plus)
        dm_min, dm_max = np.min(d_minus), np.max(d_minus)
        q = []
        for i in range(len(robots)):
            t1 = (d_plus[i] - dp_min) / (dp_max - dp_min) if dp_max != dp_min else 0.0
            t2 = (dm_max - d_minus[i]) / (dm_max - dm_min) if dm_max != dm_min else 0.0
            q.append(float(self.v * t1 + (1.0 - self.v) * t2))
        return q

    def rank_robots_ahtovik(self, robots: List[RobotState], task_name: str, task_scores: np.ndarray) -> List[Tuple[RobotState, float]]:
        vals = self.compute_ahtovik_q(robots, task_scores)
        for r, val in zip(robots, vals):
            r.last_q[task_name] = float(val)
        return sorted(zip(robots, vals), key=lambda pair: pair[1])

    def build_slots(self) -> List[TaskSlot]:
        slots: List[TaskSlot] = []
        for task_name, info in self.tasks.items():
            for r_type, count in info['requirements'].items():
                for idx in range(count):
                    slots.append(TaskSlot(f'{task_name}|{r_type}|{idx}', task_name, r_type, idx))
        return slots

    def cbba_bid(self, r: RobotState, slot: TaskSlot, bundle_position: int = 0) -> float:
        task_scores = self.tasks[slot.task_name]['scores']
        base = self.utility(r, task_scores)
        health_risk = (1.0 - r.health) * 0.03 + (1.0 - r.battery / 100.0) * 0.02
        travel_cost = self.cbba_distance_penalty * r.distance
        marginal_decay = 0.01 * bundle_position
        return base - health_risk - travel_cost - marginal_decay

    def allocate_cbba(self, available: List[RobotState]) -> Tuple[Dict[str, List[str]], Dict[str, Dict[str, float]], Dict[str, float], Dict[str, int], List[str]]:
        """Simplified CBBA consensus simulator.

        Each robot builds a bundle of feasible slots by marginal bid. At every
        round, conflicts are resolved by highest bid, ties by robot id. If a
        robot loses a slot, later bundle items are released and it rebuilds.
        This captures the key validation cost of CBBA: consensus rounds and
        message exchange before a stable assignment emerges.
        """
        slots = self.build_slots()
        feasible = {s.slot_id: [r for r in available if r.robot_type == s.robot_type] for s in slots}
        slot_by_id = {s.slot_id: s for s in slots}
        bundles: Dict[str, List[str]] = {r.robot_id: [] for r in available}
        winners: Dict[str, Optional[str]] = {s.slot_id: None for s in slots}
        winning_bids: Dict[str, float] = {s.slot_id: -math.inf for s in slots}
        q_snapshot: Dict[str, Dict[str, float]] = {task: {} for task in self.tasks.keys()}
        output_lines: List[str] = []
        changed = True
        rounds = 0

        while changed and rounds < self.cbba_max_rounds:
            rounds += 1
            changed = False
            proposals: Dict[str, List[Tuple[str, float]]] = {s.slot_id: [] for s in slots}

            for r in available:
                # release invalid bundle items where another robot currently wins
                valid_prefix = []
                for slot_id in bundles[r.robot_id]:
                    if winners.get(slot_id) in (None, r.robot_id):
                        valid_prefix.append(slot_id)
                    else:
                        break
                if valid_prefix != bundles[r.robot_id]:
                    bundles[r.robot_id] = valid_prefix
                    changed = True

                while len(bundles[r.robot_id]) < self.cbba_bundle_size:
                    best_slot = None
                    best_bid = -math.inf
                    for s in slots:
                        if s.slot_id in bundles[r.robot_id]:
                            continue
                        if r not in feasible[s.slot_id]:
                            continue
                        bid = self.cbba_bid(r, s, len(bundles[r.robot_id]))
                        current_winner = winners[s.slot_id]
                        current_bid = winning_bids[s.slot_id]
                        can_challenge = current_winner is None or bid > current_bid + 1e-12 or (abs(bid - current_bid) <= 1e-12 and r.robot_id < current_winner)
                        if can_challenge and bid > best_bid:
                            best_slot = s.slot_id
                            best_bid = bid
                    if best_slot is None:
                        break
                    bundles[r.robot_id].append(best_slot)
                    proposals[best_slot].append((r.robot_id, best_bid))

            for slot_id, slot_props in proposals.items():
                if not slot_props:
                    continue
                slot_props.sort(key=lambda x: (-x[1], x[0]))
                new_winner, new_bid = slot_props[0]
                if winners[slot_id] != new_winner:
                    winners[slot_id] = new_winner
                    winning_bids[slot_id] = new_bid
                    changed = True

        assignment: Dict[str, List[str]] = {task: [] for task in self.tasks.keys()}
        # One robot can hold only one final slot to keep fair comparison with AhToVik team synthesis.
        used = set()
        for slot in slots:
            winner = winners[slot.slot_id]
            if winner is not None and winner not in used:
                assignment[slot.task_name].append(winner)
                used.add(winner)
                q_snapshot[slot.task_name][winner] = float(-winning_bids[slot.slot_id])  # lower is better display

        messages = rounds * max(1, len(available)) * max(1, len(slots))
        cbba_metrics = {
            'cbba_consensus_rounds': rounds,
            'cbba_message_count': messages,
            'cbba_converged': int(not changed),
            'cbba_slots': len(slots),
        }
        for task, ids in assignment.items():
            output_lines.append(f'TASK: {task}')
            for idx, rid in enumerate(ids):
                role = 'PRIMARY' if idx == 0 else 'STANDBY'
                output_lines.append(f'  {role}: {rid}, cbba_bid_rank={q_snapshot[task].get(rid, math.inf):.4f}')
            if not ids:
                output_lines.append('  STATUS: PARTIAL_OR_FAILED')
        return assignment, q_snapshot, cbba_metrics, {}, output_lines

    def changed_slots(self, current: Dict[str, List[str]]) -> Tuple[int, int]:
        changed, total = 0, 0
        for task, robots in current.items():
            old = self.previous_assignments.get(task, [])
            total += max(len(robots), len(old))
            for idx in range(max(len(robots), len(old))):
                new_id = robots[idx] if idx < len(robots) else None
                old_id = old[idx] if idx < len(old) else None
                if new_id != old_id:
                    changed += 1
        return changed, max(total, 1)

    def build_metrics_and_publish(self, assignment_dict, q_snapshot, output, t0, extra_metrics=None):
        extra_metrics = extra_metrics or {}
        changed, total_slots = self.changed_slots(assignment_dict)
        self.total_reallocations += changed
        churn_rate = 100.0 * changed / total_slots
        stability_rate = max(0.0, 100.0 - churn_rate)
        self.previous_assignments = assignment_dict
        self.allocation_count += 1
        allocation_runtime_ms = (time.time() - t0) * 1000.0

        recovery_times = []
        for failed_robot, fail_t in list(self.failure_times.items()):
            still_assigned = any(failed_robot in ids for ids in assignment_dict.values())
            if not still_assigned and failed_robot not in self.recovered_failures:
                rt = time.time() - fail_t
                self.recovered_failures[failed_robot] = rt
                recovery_times.append(rt)

        human = String(); human.data = '\n'.join(output)
        self.assignment_pub.publish(human)

        required_by_task = {task: sum(info['requirements'].values()) for task, info in self.tasks.items()}
        completed_tasks = sum(1 for task, ids in assignment_dict.items() if len(ids) >= required_by_task[task])
        metrics = {
            'timestamp': time.time(),
            'elapsed_s': time.time() - self.start_time,
            'experiment_id': self.experiment_id,
            'run_id': self.run_id,
            'algorithm': self.algorithm,
            'noise_level': self.noise_level,
            'threshold': self.threshold,
            'v': self.v,
            'available_robots': len([r for r in self.robots.values() if r.available]),
            'completed_tasks': completed_tasks,
            'total_tasks': len(self.tasks),
            'changed_slots': changed,
            'total_slots': total_slots,
            'churn_rate': churn_rate,
            'stability_rate': stability_rate,
            'total_reallocations': self.total_reallocations,
            'allocation_runtime_ms': allocation_runtime_ms,
            'recovery_times_s': recovery_times,
            'assignments': assignment_dict,
            'q_snapshot': q_snapshot,
        }
        metrics.update(extra_metrics)
        msg = String(); msg.data = json.dumps(metrics)
        self.metrics_pub.publish(msg)
        self.get_logger().info('\n' + human.data)
        self.get_logger().info(f'METRICS {msg.data}')

    def allocate_tasks(self, force: bool = False):
        t0 = time.time()
        available = [r for r in self.robots.values() if r.available]
        if not available:
            self.get_logger().warn('No available robots yet.')
            return

        if self.algorithm == 'cbba':
            assignment_dict, q_snapshot, cbba_metrics, _, output = self.allocate_cbba(available)
            self.build_metrics_and_publish(assignment_dict, q_snapshot, output, t0, cbba_metrics)
            return

        used = set()
        output = []
        assignment_dict: Dict[str, List[str]] = {}
        q_snapshot: Dict[str, Dict[str, float]] = {}
        for task_name, info in self.tasks.items():
            ranked_pairs = self.rank_robots_ahtovik(available, task_name, info['scores'])
            ranked = [p[0] for p in ranked_pairs]
            q_snapshot[task_name] = {r.robot_id: float(v) for r, v in ranked_pairs}
            output.append(f'TASK: {task_name}')
            team = []
            for r_type, count in info['requirements'].items():
                candidates = [r for r in ranked if r.robot_type == r_type and r.robot_id not in used]
                selected = candidates[:count]
                for s in selected:
                    used.add(s.robot_id)
                team.extend(selected)
            required_count = sum(info['requirements'].values())
            if len(team) < required_count:
                output.append('  STATUS: PARTIAL_OR_FAILED')
            assignment_dict[task_name] = [r.robot_id for r in team]
            for idx, r in enumerate(team):
                role = 'PRIMARY' if idx == 0 else 'STANDBY'
                output.append(f'  {role}: {r.robot_id}, type={r.robot_type}, Q={r.last_q.get(task_name, math.inf):.4f}')
        self.build_metrics_and_publish(assignment_dict, q_snapshot, output, t0, {
            'cbba_consensus_rounds': 0,
            'cbba_message_count': 0,
            'cbba_converged': 1,
            'cbba_slots': 0,
        })


def main(args=None):
    rclpy.init(args=args)
    node = AhToVikAllocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
