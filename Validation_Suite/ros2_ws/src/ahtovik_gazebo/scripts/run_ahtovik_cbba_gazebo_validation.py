#!/usr/bin/env python3
"""Run Gazebo/ROS2 validation comparing AhToVik and CBBA.

Examples:
  python3 src/ahtovik_gazebo/scripts/run_ahtovik_cbba_gazebo_validation.py --quick --clean
  python3 src/ahtovik_gazebo/scripts/run_ahtovik_cbba_gazebo_validation.py --runs 10 --duration 40 --clean
"""
import argparse
import itertools
import os
import signal
import subprocess
import time
from pathlib import Path


def run_trial(params, duration_s):
    cmd = ['ros2', 'launch', 'ahtovik_gazebo', 'validation.launch.py']
    cmd.extend([f'{k}:={v}' for k, v in params.items()])
    print('\n=== RUNNING ===')
    print(' '.join(cmd))
    proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
    try:
        time.sleep(duration_s)
    finally:
        print('Stopping trial...')
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runs', type=int, default=5)
    parser.add_argument('--duration', type=int, default=35)
    parser.add_argument('--output-dir', default=str(Path.home() / 'ahtovik_cbba_validation_results'))
    parser.add_argument('--quick', action='store_true')
    parser.add_argument('--clean', action='store_true')
    args = parser.parse_args()

    out = Path(args.output_dir).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    if args.clean:
        for f in out.glob('*.csv'):
            f.unlink()
        analysis = out / 'analysis'
        if analysis.exists():
            for f in analysis.glob('*'):
                f.unlink()

    algorithms = ['ahtovik', 'cbba']
    noises = [0.01, 0.05] if args.quick else [0.01, 0.05, 0.10]
    scales = [(3, 4, 3)] if args.quick else [(3, 4, 3), (8, 10, 7), (15, 20, 15)]
    runs = 1 if args.quick else args.runs
    run_id = 0

    # 1) Perturbation stability and churn comparison.
    for algorithm, noise, rep in itertools.product(algorithms, noises, range(runs)):
        run_id += 1
        run_trial({
            'experiment_id': 'ahtovik_vs_cbba_noise',
            'run_id': run_id,
            'algorithm': algorithm,
            'noise_level': noise,
            'threshold': 0.10,
            'v': 0.50,
            'seed': 42 + rep,
            'failure_enabled': 'true',
            'fail_robot': 'uav_0',
            'fail_time': 12.0,
            'output_dir': args.output_dir,
        }, args.duration)

    # 2) Runtime and communication/consensus cost under fleet-size scaling.
    for algorithm, (d, g, c), rep in itertools.product(algorithms, scales, range(runs)):
        run_id += 1
        run_trial({
            'experiment_id': 'ahtovik_vs_cbba_scalability',
            'run_id': run_id,
            'algorithm': algorithm,
            'noise_level': 0.05,
            'threshold': 0.10,
            'v': 0.50,
            'seed': 200 + rep,
            'num_drones': d,
            'num_ground': g,
            'num_creepers': c,
            'failure_enabled': 'true',
            'fail_robot': 'uav_0',
            'fail_time': 12.0,
            'output_dir': args.output_dir,
        }, args.duration)

    print(f'AhToVik vs CBBA campaign complete. Results in {args.output_dir}')
    print('Analyze with:')
    print(f'  python3 src/ahtovik_gazebo/scripts/analyze_validation_results.py --input-dir {args.output_dir}')


if __name__ == '__main__':
    main()
