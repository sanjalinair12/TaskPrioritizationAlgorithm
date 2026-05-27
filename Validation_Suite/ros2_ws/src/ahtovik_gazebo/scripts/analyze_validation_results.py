#!/usr/bin/env python3
"""Analyze AhToVik/CBBA validation CSV results and create summary tables/plots."""
import argparse
import glob
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def load_metrics(input_dir):
    frames = []
    for path in glob.glob(str(Path(input_dir).expanduser() / '*_metrics.csv')):
        try:
            df = pd.read_csv(path)
            df['source_file'] = Path(path).name
            frames.append(df)
        except Exception as exc:
            print(f'Skipping {path}: {exc}')
    if not frames:
        raise SystemExit(f'No *_metrics.csv files found in {input_dir}')
    df = pd.concat(frames, ignore_index=True)
    for col in ['cbba_consensus_rounds', 'cbba_message_count', 'cbba_converged']:
        if col not in df.columns:
            df[col] = 0
    return df


def summarize(df):
    last = df.sort_values('timestamp').groupby(['source_file'], as_index=False).tail(1)
    summary = last.groupby(['experiment_id', 'algorithm', 'noise_level', 'threshold', 'v'], dropna=False).agg(
        runs=('run_id', 'nunique'),
        stability_mean=('stability_rate', 'mean'),
        stability_std=('stability_rate', 'std'),
        churn_mean=('churn_rate', 'mean'),
        churn_std=('churn_rate', 'std'),
        realloc_mean=('total_reallocations', 'mean'),
        runtime_ms_mean=('allocation_runtime_ms', 'mean'),
        completed_tasks_mean=('completed_tasks', 'mean'),
        available_robots_mean=('available_robots', 'mean'),
        cbba_rounds_mean=('cbba_consensus_rounds', 'mean'),
        cbba_messages_mean=('cbba_message_count', 'mean'),
        cbba_converged_mean=('cbba_converged', 'mean'),
    ).reset_index()
    return last, summary


def plot_by_noise(last, outdir, metric, filename, ylabel, title):
    subset = last[last['experiment_id'].isin(['baseline_noise', 'ahtovik_vs_cbba_noise'])]
    if subset.empty:
        return
    pivot = subset.groupby(['algorithm', 'noise_level'])[metric].mean().reset_index()
    fig = plt.figure()
    for algo, g in pivot.groupby('algorithm'):
        plt.plot((g['noise_level'] * 100).to_numpy(), g[metric].to_numpy(), marker='o', label=algo)
    plt.xlabel('Perturbation noise (%)')
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend()
    plt.grid(True, alpha=0.3)
    fig.savefig(outdir / filename, dpi=200, bbox_inches='tight')
    plt.close(fig)


def plot_runtime_scalability(last, outdir):
    subset = last[last['experiment_id'].isin(['scalability', 'ahtovik_vs_cbba_scalability'])].copy()
    if subset.empty:
        return
    subset['fleet_size'] = subset['available_robots']
    g = subset.groupby(['algorithm', 'fleet_size'])['allocation_runtime_ms'].mean().reset_index()
    fig = plt.figure()
    for algo, gg in g.groupby('algorithm'):
        plt.plot(gg['fleet_size'].to_numpy(), gg['allocation_runtime_ms'].to_numpy(), marker='o', label=algo)
    plt.xlabel('Available robots')
    plt.ylabel('Mean allocation runtime (ms)')
    plt.title('AhToVik vs CBBA runtime scalability')
    plt.legend()
    plt.grid(True, alpha=0.3)
    fig.savefig(outdir / 'runtime_scalability.png', dpi=200, bbox_inches='tight')
    plt.close(fig)


def plot_cbba_message_count(last, outdir):
    subset = last[last['algorithm'] == 'cbba'].copy()
    if subset.empty:
        return
    g = subset.groupby('noise_level')[['cbba_consensus_rounds', 'cbba_message_count']].mean().reset_index()
    fig = plt.figure()
    plt.plot((g['noise_level'] * 100).to_numpy(), g['cbba_consensus_rounds'].to_numpy(), marker='o')
    plt.xlabel('Perturbation noise (%)')
    plt.ylabel('Mean CBBA consensus rounds')
    plt.title('CBBA consensus rounds under perturbation')
    plt.grid(True, alpha=0.3)
    fig.savefig(outdir / 'cbba_consensus_rounds.png', dpi=200, bbox_inches='tight')
    plt.close(fig)

    fig = plt.figure()
    plt.plot((g['noise_level'] * 100).to_numpy(), g['cbba_message_count'].to_numpy(), marker='o')
    plt.xlabel('Perturbation noise (%)')
    plt.ylabel('Mean message count')
    plt.title('CBBA communication cost under perturbation')
    plt.grid(True, alpha=0.3)
    fig.savefig(outdir / 'cbba_message_count.png', dpi=200, bbox_inches='tight')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-dir', default=str(Path.home() / 'ahtovik_validation_results'))
    parser.add_argument('--output-dir', default=None)
    args = parser.parse_args()
    input_dir = Path(args.input_dir).expanduser()
    outdir = Path(args.output_dir).expanduser() if args.output_dir else input_dir / 'analysis'
    outdir.mkdir(parents=True, exist_ok=True)

    df = load_metrics(input_dir)
    last, summary = summarize(df)
    df.to_csv(outdir / 'all_metrics_raw.csv', index=False)
    last.to_csv(outdir / 'trial_final_rows.csv', index=False)
    summary.to_csv(outdir / 'summary_table.csv', index=False)

    plot_by_noise(last, outdir, 'stability_rate', 'stability_vs_noise.png', 'Mean stability rate (%)', 'Assignment stability under telemetry perturbation')
    plot_by_noise(last, outdir, 'churn_rate', 'churn_vs_noise.png', 'Mean churn rate (%)', 'Assignment churn under telemetry perturbation')
    plot_by_noise(last, outdir, 'allocation_runtime_ms', 'runtime_vs_noise.png', 'Mean allocation runtime (ms)', 'Runtime under telemetry perturbation')
    plot_runtime_scalability(last, outdir)
    plot_cbba_message_count(last, outdir)

    print('Wrote:')
    for name in ['summary_table.csv', 'stability_vs_noise.png', 'churn_vs_noise.png', 'runtime_vs_noise.png', 'runtime_scalability.png', 'cbba_consensus_rounds.png', 'cbba_message_count.png']:
        print(outdir / name)


if __name__ == '__main__':
    main()
