#!/usr/bin/env python3
"""
Plot the odometry variants written by the trajectory_logger node.

  python3 Scripts/plot_trajectories.py Results/trajectories_11_10_48.csv

Creates Images/<csv-name>_plot.png and <csv-name>_points.csv next to the csv.

Significant points = start, every corner (turn on the spot detected from the
heading rate of /my_odom; the pose is taken 0.5 s AFTER the turn, so θ shows
the completed turn) and the end. For each point the poses of all
variants at the same time stamp are compared.
"""

import argparse
from collections import defaultdict
import csv
import math
import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# Repo-Ordnerstruktur: Scripts/ (dieses Skript), Messaufnahmen/, Results/, Images/
REPO = Path(__file__).resolve().parent.parent
BAGS, RESULTS, IMAGES = REPO / 'Messaufnahmen', REPO / 'Results', REPO / 'Images'


def bag_dir(p):
    """Bag folder as given, or by name from Messaufnahmen/ (e.g. 'subset_5minSTILL')."""
    p = Path(p)
    return p if p.exists() else BAGS / p

STYLE = {
    '/my_odom': {'color': 'tab:green', 'label': '/my_odom  (Encoder, eigene Berechnung)'},
    '/my_odom_imu': {'color': 'tab:orange', 'label': '/my_odom_imu  (Encoder + IMU-Heading)'},
    '/odom': {'color': 'tab:red', 'label': '/odom  (turtlebot3_node)'},
}
BROKEN_LIMIT = 1e3  # |x| or |y| above this [m] -> odometry diverged


def load(path):
    data = defaultdict(list)
    with open(path) as f:
        for row in csv.DictReader(f):
            data[row['topic']].append(
                [float(row['stamp']), float(row['x']), float(row['y']), float(row['yaw'])])
    out = {}
    for k, v in data.items():
        arr = np.array(v)
        arr[:, 3] = np.unwrap(arr[:, 3])  # continuous heading
        out[k] = arr
    return out


def find_turns(t, th, rate_thresh=0.3, min_turn_deg=20.0, settle=0.5):
    """Return indices `settle` seconds after each turn (= corner of the path)."""
    rate = np.gradient(th, t)
    turning = np.abs(rate) > rate_thresh
    idx, i = [], 0
    while i < len(turning):
        if turning[i]:
            j = i
            while j < len(turning) and turning[j]:
                j += 1
            if abs(math.degrees(th[min(j, len(th) - 1)] - th[i])) >= min_turn_deg:
                k = int(np.searchsorted(t, t[min(j, len(t) - 1)] + settle))
                idx.append(min(k, len(t) - 1))
            i = j
        else:
            i += 1
    return idx


def pose_at(arr, stamp):
    k = int(np.clip(np.searchsorted(arr[:, 0], stamp), 0, len(arr) - 1))
    return arr[k]


def main():
    plt.switch_backend('Agg')  # no window needed, only png files
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv')
    ap.add_argument('--image-dir', default=str(IMAGES), help='where the png goes (default: Images/)')
    ap.add_argument('--reference', default='/my_odom',
                    help='topic used to detect the corners (default /my_odom)')
    args = ap.parse_args()

    data = load(args.csv)
    base = os.path.splitext(args.csv)[0]
    os.makedirs(args.image_dir, exist_ok=True)
    img_base = os.path.join(args.image_dir, os.path.basename(base))
    valid, broken = {}, {}
    for topic, arr in data.items():
        (broken if np.nanmax(np.abs(arr[:, 1:3])) > BROKEN_LIMIT else valid)[topic] = arr
    for topic, arr in broken.items():
        print(f'WARNING {topic}: odometry diverged (x in [{arr[:, 1].min():.3g}, '
              f'{arr[:, 1].max():.3g}], yaw constant = {math.degrees(arr[0, 3]):.1f} deg: '
              f'{len(np.unique(arr[:, 3])) == 1}) -> not drawn in the x/y plot')

    ref = valid[args.reference]
    t0 = ref[0, 0]
    corners = find_turns(ref[:, 0], ref[:, 3])
    points = [('Start', ref[0, 0])] + \
        [(f'P{i + 1}', ref[k, 0]) for i, k in enumerate(corners)] + [('Ende', ref[-1, 0])]

    fig = plt.figure(figsize=(15, 7.5))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.15, 1])
    ax = fig.add_subplot(gs[:, 0])
    ax_h = fig.add_subplot(gs[0, 1])
    ax_d = fig.add_subplot(gs[1, 1])

    for topic, arr in valid.items():
        st = STYLE.get(topic, {'label': topic})
        ax.plot(arr[:, 1], arr[:, 2], lw=2, **st)
        ax.plot(arr[-1, 1], arr[-1, 2], 's', color=st.get('color'), ms=7)
    ax.plot(0, 0, 'ko', ms=7)
    for name, stamp in points:
        x, y = pose_at(ref, stamp)[1:3]
        ax.annotate(f'{name}\n{stamp - t0:.0f}s', (x, y), textcoords='offset points',
                    xytext=(6, 6), fontsize=9)
        for arr in valid.values():
            p = pose_at(arr, stamp)
            ax.plot(p[1], p[2], 'k.', ms=6)
    ax.set_title('Trajektorien (Start = Ursprung, ■ = Endpunkt)')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.axis('equal')
    ax.grid(True, alpha=0.4)
    ax.legend(loc='best', fontsize=9)
    if broken:
        ax.text(0.01, 0.01, 'nicht gezeichnet (divergiert): ' + ', '.join(broken),
                transform=ax.transAxes, fontsize=8, color='tab:red')

    for topic, arr in {**valid, **broken}.items():
        st = STYLE.get(topic, {'label': topic})
        ax_h.plot(arr[:, 0] - t0, np.degrees(arr[:, 3]), lw=1.8, **st)
    for _, stamp in points[1:-1]:
        ax_h.axvline(stamp - t0, color='gray', lw=0.6, ls=':')
    ax_h.set_title('Orientierung θ (unwrapped, gepunktet = signifikante Punkte)')
    ax_h.set_xlabel('t [s]')
    ax_h.set_ylabel('θ [°]')
    ax_h.grid(True, alpha=0.4)
    ax_h.legend(fontsize=8)

    others = [k for k in valid if k != args.reference]
    for topic in others:
        arr = valid[topic]
        r = np.array([pose_at(ref, s)[1:3] for s in arr[:, 0]])
        ax_d.plot(arr[:, 0] - t0, np.hypot(arr[:, 1] - r[:, 0], arr[:, 2] - r[:, 1]) * 100,
                  color=STYLE.get(topic, {}).get('color'), lw=1.8,
                  label=f'|{topic} − {args.reference}|')
    ax_d.set_title('Positionsabweichung zwischen den Varianten')
    ax_d.set_xlabel('t [s]')
    ax_d.set_ylabel('Abstand [cm]')
    ax_d.grid(True, alpha=0.4)
    ax_d.legend(fontsize=8)

    fig.suptitle(os.path.basename(args.csv))
    fig.tight_layout()
    fig.savefig(img_base + '_plot.png', dpi=150)

    # table of significant points
    rows = []
    for name, stamp in points:
        row = {'punkt': name, 't [s]': f'{stamp - t0:.1f}'}
        for topic, arr in valid.items():
            p = pose_at(arr, stamp)
            row[f'{topic} x'] = f'{p[1]:.3f}'
            row[f'{topic} y'] = f'{p[2]:.3f}'
            row[f'{topic} θ°'] = f'{math.degrees(p[3]):.1f}'
        for topic in others:
            a, b = pose_at(valid[topic], stamp), pose_at(ref, stamp)
            row[f'Δ {topic} [cm]'] = f'{math.hypot(a[1] - b[1], a[2] - b[2]) * 100:.1f}'
        rows.append(row)
    with open(base + '_points.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    print(f'\n{len(corners)} Ecken erkannt. Signifikante Punkte:')
    keys = list(rows[0])
    print(' | '.join(keys))
    for row in rows:
        print(' | '.join(row[k] for k in keys))
    for topic, arr in valid.items():
        seg = np.hypot(np.diff(arr[:, 1]), np.diff(arr[:, 2])).sum()
        print(f'{topic}: Weglänge {seg:.2f} m, Gesamtdrehung '
              f'{math.degrees(arr[-1, 3] - arr[0, 3]):.1f}°')
    print(f'\n-> {img_base}_plot.png\n-> {base}_points.csv')


if __name__ == '__main__':
    main()
