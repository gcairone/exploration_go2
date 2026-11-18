#!/usr/bin/env python3
"""Summarise a batch of exploration runs.

    python3 analyze.py <batch_dir> [--plot coverage.png] [--csv summary.csv]

Per run (metrics.csv, done.txt, goal_manager.log):
  t90 / t95   simulated time to know 90 % / 95 % of the world interior (cells >= 0.15 m from walls)
  coverage    final known fraction of the interior
  goals       goals sent by goal_manager;  end = coverage | stall | timeout
A run is successful when it reaches 95 % coverage. Times are mean +- std over successful runs.
Prints a markdown table grouped by config / world / metric.
"""
import argparse
import csv
import os
import re
import statistics as st

import numpy as np

WORLDS = ['tb3_world', 'square', 'corridor', 'corridor_taper', 'four_rooms', 'three_rooms']
METRICS = ['euclidean', 'pseudo_info_gain', 'dim_cluster']


def load(batch):
    runs = []
    for name in sorted(os.listdir(batch)):
        d = os.path.join(batch, name)
        if not os.path.isfile(os.path.join(d, 'done.txt')):
            continue
        info = dict(kv.split('=') for kv in open(os.path.join(d, 'run.txt')).readline().split())
        M = np.genfromtxt(os.path.join(d, 'metrics.csv'), delimiter=',', names=True)
        cov = np.maximum.accumulate(np.nan_to_num(M['coverage']))
        k0 = np.argmax(M['known_m2'] > 0)  # samples before the first (latched) /map message
        cov[:k0] = cov[k0]
        log = open(os.path.join(d, 'goal_manager.log'), errors='ignore').read()
        r = dict(name=name, world=info['world'], metric=info['metric'], config=info['config'],
                 end=open(os.path.join(d, 'done.txt')).read().split()[0], coverage=float(cov[-1]),
                 goals=len(re.findall('Invio goal', log)), aborted=len(re.findall('Goal abortito', log)),
                 t=M['sim_s'], cov=cov)
        for q in (90, 95):
            k = np.nonzero(cov >= q / 100)[0]
            r[f't{q}'] = float(M['sim_s'][k[0]]) if len(k) else None
            r[f'd{q}'] = float(M['dist_m'][k[0]]) if len(k) else None
        runs.append(r)
    return runs


def fmt(v):
    v = [x for x in v if x is not None]
    if not v:
        return '–'
    return f'{st.mean(v):.0f} ± {st.stdev(v):.0f}' if len(v) > 1 else f'{v[0]:.0f}'


def table(runs):
    print('| config | world | metric | success | t90 [s] | t95 [s] | final coverage [%] | goals |')
    print('|---|---|---|---|---|---|---|---|')
    for c in sorted({r['config'] for r in runs}):
        for w in WORLDS:
            for m in METRICS:
                R = [r for r in runs if (r['config'], r['world'], r['metric']) == (c, w, m)]
                if not R:
                    continue
                ok = [r for r in R if r['t95'] is not None]
                cov = [100 * r['coverage'] for r in R]
                print(f"| {c} | {w} | {m} | {len(ok)}/{len(R)} | {fmt([r['t90'] for r in ok])} | "
                      f"{fmt([r['t95'] for r in ok])} | {st.mean(cov):.1f} ± {st.pstdev(cov):.1f} | "
                      f"{st.mean(r['goals'] for r in R):.1f} |")


def settle_time(r, eps=0.005):
    """Last time the coverage grew by more than eps."""
    k = np.nonzero(r['cov'] < r['cov'][-1] - eps)[0]
    return float(r['t'][k[-1] + 1]) if len(k) else float(r['t'][0])


def plot(runs, out, config, baseline='original'):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors = {'euclidean': '#2a78d6', 'pseudo_info_gain': '#eb6834', 'dim_cluster': '#1baf7a'}
    ws = [w for w in WORLDS if any(r['world'] == w and r['config'] == config for r in runs)]
    fig, axs = plt.subplots(2, 3, figsize=(12, 6.4), sharey=True, facecolor='#fcfcfb')
    for ax, w in zip(axs.flat, ws):
        ax.set_facecolor('#fcfcfb')
        R = [r for r in runs if r['world'] == w and r['config'] == config]
        B = [r for r in runs if r['world'] == w and r['config'] == baseline]
        tmax = max(30.0, 1.15 * max(settle_time(r) for r in R + B))
        T = np.arange(0, tmax + 1, 1.0)

        def band(S):
            C = np.array([np.interp(T, r['t'], 100 * r['cov'], right=100 * r['cov'][-1]) for r in S])
            return np.percentile(C, [25, 50, 75], axis=0)
        for m in METRICS:
            S = [r for r in R if r['metric'] == m]
            if S:
                q1, med, q3 = band(S)
                ax.fill_between(T, q1, q3, color=colors[m], alpha=.15, lw=0)
                ax.plot(T, med, color=colors[m], lw=2, label=m)
        if B:
            q1, med, q3 = band(B)
            ax.fill_between(T, q1, q3, color='#7a7974', alpha=.10, lw=0)
            ax.plot(T, med, color='#7a7974', lw=1.6, ls=(0, (4, 2)), label=f'{baseline} config (euclidean)')
        ax.axhline(95, color='#a3a29b', lw=.8, ls=(0, (4, 3)))
        ax.set_title(w, fontsize=10, color='#0b0b0b', loc='left')
        ax.set_ylim(0, 101)
        ax.set_xlim(0, tmax)
        ax.grid(True, color='#e6e5e0', lw=.6)
        for s in ('top', 'right'):
            ax.spines[s].set_visible(False)
        for s in ('left', 'bottom'):
            ax.spines[s].set_color('#c3c2b7')
        ax.tick_params(colors='#52514e', labelsize=8)
    for ax in axs[:, 0]:
        ax.set_ylabel('interior known [%]', color='#52514e', fontsize=9)
    for ax in axs[-1]:
        ax.set_xlabel('simulated time [s]', color='#52514e', fontsize=9)
    h, lab = axs.flat[0].get_legend_handles_labels()
    fig.legend(h, lab, loc='upper center', ncol=4, frameon=False, fontsize=9,
               title=f'{config} config: median and interquartile range per frontier metric', title_fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, .92))
    fig.savefig(out, dpi=130, facecolor=fig.get_facecolor())


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('batch')
    p.add_argument('--plot')
    p.add_argument('--plot-config', default='longrange')
    p.add_argument('--csv')
    a = p.parse_args()
    runs = load(a.batch)
    table(runs)
    if a.csv:
        keys = ['name', 'config', 'world', 'metric', 'end', 'coverage', 't90', 't95', 'd90', 'd95', 'goals', 'aborted']
        with open(a.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(keys)
            for r in runs:
                w.writerow([round(r[k], 4) if isinstance(r[k], float) else r[k] for k in keys])
    if a.plot:
        plot(runs, a.plot, a.plot_config)
