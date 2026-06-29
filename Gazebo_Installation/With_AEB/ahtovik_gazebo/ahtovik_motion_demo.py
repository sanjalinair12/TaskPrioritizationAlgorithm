#!/usr/bin/env python3
"""Animated AhToVik 30-robot demo for Gazebo Classic + ROS 2 Humble.

Visual behavior:
- 10 drones D1-D10, 10 ground robots G1-G10, 10 creepers C1-C10.
- Three highlighted task sites: scan, supply, lifting.
- Active robots move toward their assigned task sites.
- At selected frames, active agents fail and freeze.
- Standby agents with next-best Q values take over and move to the same task site.
- Console prints Q values and assignments for video narration.
"""
import math, random, time, csv, os
from dataclasses import dataclass
from typing import Dict, Tuple, List
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose

@dataclass
class Agent:
    name: str
    kind: str
    x: float
    y: float
    z: float
    speed: float
    failed: bool = False
    task: str = "idle"
    active: bool = False
    q: float = 1.0

TASKS = {
    "Scan and Rescue": (20.0, 80.0, 1.6),
    "Supply Necessities": (80.0, 80.0, 0.45),
    "Lifting Remains": (50.0, 20.0, 0.30),
}
REQUIREMENTS = {
    "Scan and Rescue": {"D": 1, "C": 1},
    "Supply Necessities": {"D": 1, "G": 1},
    "Lifting Remains": {"G": 1, "C": 1},
}
FAILURE_SCHEDULE = {
    80: ["D1"],
    150: ["C1"],
    220: ["G1"],
    300: ["D2", "C2"],
}

class AhToVikMotionDemo(Node):
    def __init__(self):
        super().__init__('ahtovik_motion_demo')
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.get_logger().info('Waiting for /gazebo/set_entity_state ...')
        self.client.wait_for_service(timeout_sec=30.0)
        self.frame = 0
        self.rng = random.Random(42)
        self.agents: Dict[str, Agent] = {}
        self._make_agents()
        self.assignments: Dict[str, List[str]] = {}
        self.log_path = os.path.expanduser('~/ahtovik_30_robot_motion_log.csv')
        with open(self.log_path, 'w', newline='') as f:
            csv.writer(f).writerow(['frame','task','assigned_agents','failed_agents','mean_q'])
        self._compute_assignments()
        self.timer = self.create_timer(0.15, self.step)

    def _make_agents(self):
        for i in range(10):
            self.agents[f'D{i+1}'] = Agent(f'D{i+1}', 'D', 5+i*4, 88-(i%3)*6, 1.5, 1.45)
        for i in range(10):
            self.agents[f'G{i+1}'] = Agent(f'G{i+1}', 'G', 5+i*5, 45+(i%4)*5, 0.4, 0.75)
        for i in range(10):
            self.agents[f'C{i+1}'] = Agent(f'C{i+1}', 'C', 8+i*4, 18+(i%3)*6, 0.25, 0.55)

    def _q_value(self, a: Agent, task: str) -> float:
        tx, ty, _ = TASKS[task]
        dist = math.hypot(a.x - tx, a.y - ty) / 120.0
        health_noise = self.rng.uniform(0.0, 0.08)
        type_bonus = 0.0
        if task == 'Scan and Rescue' and a.kind == 'D': type_bonus = -0.08
        if task == 'Supply Necessities' and a.kind == 'G': type_bonus = -0.06
        if task == 'Lifting Remains' and a.kind in ('G','C'): type_bonus = -0.05
        return max(0.01, min(1.0, dist + health_noise + type_bonus))

    def _compute_assignments(self):
        for a in self.agents.values():
            a.task, a.active = 'idle', False
        self.assignments = {}
        used=set()
        for task, req in REQUIREMENTS.items():
            selected=[]
            for kind, count in req.items():
                pool=[a for a in self.agents.values() if a.kind==kind and not a.failed and a.name not in used]
                for a in pool: a.q = self._q_value(a, task)
                pool.sort(key=lambda x: x.q)
                for a in pool[:count]:
                    a.task, a.active = task, True
                    selected.append(a.name); used.add(a.name)
            self.assignments[task]=selected

    def _move_towards_target(self, a: Agent):
        if a.failed or not a.active or a.task == 'idle':
            return
        tx, ty, tz = TASKS[a.task]
        dx, dy = tx-a.x, ty-a.y
        d = math.hypot(dx, dy)
        if d > 0.15:
            step = min(a.speed, d)
            a.x += step * dx/d
            a.y += step * dy/d
        # drones float; ground/creepers stay low
        if a.kind == 'D':
            a.z = 1.8 + 0.25*math.sin(self.frame/8.0)
        elif a.kind == 'G':
            a.z = 0.4
        else:
            a.z = 0.25

    def _set_pose(self, a: Agent):
        req = SetEntityState.Request()
        state = EntityState()
        state.name = a.name
        state.reference_frame = 'world'
        state.pose.position.x = float(a.x)
        state.pose.position.y = float(a.y)
        state.pose.position.z = float(a.z)
        # rotate failed agents slightly so they look visibly stalled
        if a.failed:
            state.pose.orientation.z = 0.707
            state.pose.orientation.w = 0.707
        else:
            state.pose.orientation.w = 1.0
        req.state = state
        self.client.call_async(req)

    def step(self):
        self.frame += 1
        if self.frame in FAILURE_SCHEDULE:
            for name in FAILURE_SCHEDULE[self.frame]:
                if name in self.agents:
                    self.agents[name].failed = True
                    self.get_logger().warn(f'FAILURE injected: {name}. Standby takeover triggered.')
            self._compute_assignments()

        # mild Q perturbation and occasional reassignment threshold check
        if self.frame % 55 == 0:
            previous = {k: tuple(v) for k,v in self.assignments.items()}
            self._compute_assignments()
            if previous != {k: tuple(v) for k,v in self.assignments.items()}:
                self.get_logger().info('AhToVik threshold breach: reassignment executed.')

        for a in self.agents.values():
            self._move_towards_target(a)
            self._set_pose(a)

        if self.frame % 10 == 0:
            failed = [a.name for a in self.agents.values() if a.failed]
            print('\n' + '='*64)
            print(f'AhToVik animated Gazebo frame {self.frame}')
            print('Task sites: magenta=Scan, orange=Supply, cyan=Lifting')
            print('Failed agents:', ', '.join(failed) if failed else 'none')
            for task, names in self.assignments.items():
                txt = ', '.join([f'{n}(Q={self.agents[n].q:.3f})' for n in names])
                print(f'  {task}: {txt}')
            with open(self.log_path, 'a', newline='') as f:
                writer=csv.writer(f)
                for task,names in self.assignments.items():
                    qs=[self.agents[n].q for n in names]
                    writer.writerow([self.frame, task, ' '.join(names), ' '.join(failed), sum(qs)/len(qs) if qs else 0])
            print(f'Log: {self.log_path}')


def main():
    rclpy.init()
    node = AhToVikMotionDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
