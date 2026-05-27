#!/usr/bin/env python3
"""Run a repeatable AhToVik validation campaign.

Usage after building/sourcing the workspace:
  python3 src/ahtovik_gazebo/scripts/run_validation_campaign.py --quick
  python3 src/ahtovik_gazebo/scripts/run_validation_campaign.py --runs 10 --duration 40

This script launches ROS2 trials and terminates each trial after the requested duration.
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
    parser.add_argument('--runs', type=int, default=3)
    parser.add_argument('--duration', type=int, default=35)
    parser.add_argument('--output-dir', default=str(Path.home() / 'ahtovik_validation_results'))
    parser.add_argument('--quick', action='store_true', help='small smoke-test campaign with all baselines')
    parser.add_argument('--baselines-only', action='store_true', help='run only the baseline/noise comparison')
    parser.add_argument('--clean', action='store_true', help='remove existing csv files from output directory before running')
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

    if args.quick:
        algorithms = ['ahtovik', 'saw', 'topsis', 'auction', 'cbba']
        noises = [0.01, 0.05]
        thresholds = [0.10]
        vs = [0.5]
        scales = [(3, 4, 3)]
        runs = 1
    else:
        algorithms = ['ahtovik', 'saw', 'topsis', 'auction', 'cbba']
        noises = [0.01, 0.05, 0.10]
        thresholds = [0.05, 0.10, 0.15]
        vs = [0.25, 0.50, 0.75]
        scales = [(3, 4, 3), (8, 10, 7), (15, 20, 15)]
        runs = args.runs

    run_id = 0
    # Baseline algorithm/noise comparison at default threshold/v.
    for algorithm, noise, rep in itertools.product(algorithms, noises, range(runs)):
        run_id += 1
        run_trial({
            'experiment_id': 'baseline_noise',
            'run_id': run_id,
            'algorithm': algorithm,
            'noise_level': noise,
            'threshold': 0.10,
            'v': 0.50,
            'seed': 42 + rep,
            'output_dir': args.output_dir,
        }, args.duration)

    if args.baselines_only:
        print(f'Baseline comparison complete. Results in {args.output_dir}')
        return

    # Threshold sensitivity only for AhToVik.
    for threshold, rep in itertools.product(thresholds, range(runs)):
        run_id += 1
        run_trial({
            'experiment_id': 'threshold_sensitivity',
            'run_id': run_id,
            'algorithm': 'ahtovik',
            'noise_level': 0.05,
            'threshold': threshold,
            'v': 0.50,
            'seed': 100 + rep,
            'output_dir': args.output_dir,
        }, args.duration)

    # v sensitivity only for AhToVik.
    for v, rep in itertools.product(vs, range(runs)):
        run_id += 1
        run_trial({
            'experiment_id': 'v_sensitivity',
            'run_id': run_id,
            'algorithm': 'ahtovik',
            'noise_level': 0.05,
            'threshold': 0.10,
            'v': v,
            'seed': 200 + rep,
            'output_dir': args.output_dir,
        }, args.duration)

    # Scalability with logical robot counts.
    for idx, (d, g, c) in enumerate(scales):
        for rep in range(runs):
            run_id += 1
            run_trial({
                'experiment_id': 'scalability',
                'run_id': run_id,
                'algorithm': 'ahtovik',
                'noise_level': 0.05,
                'threshold': 0.10,
                'v': 0.50,
                'num_drones': d,
                'num_ground': g,
                'num_creepers': c,
                'seed': 300 + idx * 10 + rep,
                'output_dir': args.output_dir,
            }, args.duration)

    print(f'Campaign complete. Results in {args.output_dir}')


if __name__ == '__main__':
    main()
