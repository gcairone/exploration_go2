#!/usr/bin/env python3
"""Write a job list for run_batch.sh: every start pose of sim/starts/<world>.csv x metrics x configs.

    python3 make_jobs.py --worlds square,three_rooms --metrics euclidean,dim_cluster \
        --configs longrange [--starts 8] [--timeout 600] > jobs.csv
"""
import argparse
import csv
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SHORT = {'euclidean': 'eu', 'pseudo_info_gain': 'pig', 'dim_cluster': 'dc'}
WORLDS = ['tb3_world', 'square', 'corridor', 'corridor_taper', 'four_rooms', 'three_rooms']

p = argparse.ArgumentParser()
p.add_argument('--worlds', default=','.join(WORLDS))
p.add_argument('--metrics', default=','.join(SHORT))
p.add_argument('--configs', default='longrange')
p.add_argument('--starts', type=int, default=8, help='first N start poses of each world')
p.add_argument('--timeout', type=int, default=600)
a = p.parse_args()
for c in a.configs.split(','):
    for w in a.worlds.split(','):
        rows = list(csv.DictReader(open(os.path.join(HERE, '..', 'starts', f'{w}.csv'))))[:a.starts]
        for r in rows:
            for m in a.metrics.split(','):
                print(f"{c}_{w}_{SHORT[m]}_{r['i']},{w},{r['x']},{r['y']},{r['yaw']},{m},{c},{a.timeout}")
