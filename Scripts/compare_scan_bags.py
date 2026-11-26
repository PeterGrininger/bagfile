#!/usr/bin/env python3
"""
Compare two static laser bags where the robot was rotated in place between them
(e.g. subset_5minSTILL and subset_5min_90d).

  source /opt/ros/jazzy/setup.bash
  python3 Scripts/compare_scan_bags.py subset_5minSTILL subset_5min_90d \
      --labels "Ausgangslage (5minSTILL)" "um 90° gedreht (5min_90d)" --out Images/vergleich_5min

Question: are the directions with a large standard deviation a property of the
SENSOR (bad beam index -> stays at the same index after rotating) or of the
ENVIRONMENT (surface / edge / distance -> moves with the scene)?
The rotation between the bags is estimated from the mean range profiles
(circular shift with the smallest mean absolute difference).

Creates <out>_ausrichtung.png and <out>_sigma_intensitaet.png and prints the
numbers used in the report.
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

# Repo-Ordnerstruktur: Scripts/ (dieses Skript), Messaufnahmen/, Results/, Images/
REPO = Path(__file__).resolve().parent.parent
BAGS, RESULTS, IMAGES = REPO / 'Messaufnahmen', REPO / 'Results', REPO / 'Images'


def bag_dir(p):
    """Bag folder as given, or by name from Messaufnahmen/ (e.g. 'subset_5minSTILL')."""
    p = Path(p)
    return p if p.exists() else BAGS / p

BLUE, ORANGE = '#257de0', '#f47230'


def load(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
    rng, inten, rmin, rmax = [], [], None, None
    while r.has_next():
        _, data, _ = r.read_next()
        m = deserialize_message(data, LaserScan)
        rng.append(np.asarray(m.ranges, float))
        inten.append(np.asarray(m.intensities, float))
        rmin, rmax = m.range_min, m.range_max
    rng, inten = np.vstack(rng), np.vstack(inten)
    ok = np.isfinite(rng) & (rng >= rmin) & (rng <= rmax)
    x = np.where(ok, rng, np.nan)
    with np.errstate(all='ignore'):
        return {
            'n': len(rng),
            'mean': np.nanmean(x, 0),
            'std': np.nanstd(x, 0, ddof=1),
            'intensity': np.nanmean(np.where(ok, inten, np.nan), 0),
            'valid': ok.mean(0),
        }


def best_shift(a, b):
    """k so that b[i] ~ a[i - k] (scene appears k beams later in bag b)."""
    err = [np.nanmean(np.abs(np.roll(a, k) - b)) for k in range(len(a))]
    k = int(np.argmin(err))
    return k, err[k]


def corr(a, b):
    m = np.isfinite(a) & np.isfinite(b)
    return float(np.corrcoef(a[m], b[m])[0, 1])


def main():
    plt.switch_backend('Agg')
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_a')
    ap.add_argument('bag_b')
    ap.add_argument('--labels', nargs=2, default=['Bag A', 'Bag B'])
    ap.add_argument('--out', default=str(IMAGES / 'vergleich_5min'),
                    help='output prefix for the pngs (default: Images/vergleich_5min)')
    args = ap.parse_args()
    la, lb = args.labels

    A, B = load(bag_dir(args.bag_a)), load(bag_dir(args.bag_b))
    k, err = best_shift(A['mean'], B['mean'])
    lsa, lsb = np.log10(A['std']), np.log10(B['std'])
    c_sensor, c_env = corr(lsa, lsb), corr(np.roll(lsa, k), lsb)
    rot = k if k <= 180 else k - 360   # scene shift in degrees (1 beam = 1°)

    ang = np.arange(360)
    fig, axs = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    ax = axs[0]
    ax.plot(ang, A['mean'], color=BLUE, lw=1.5, label=f'{la} ({A["n"]} Scans)')
    ax.plot(ang, B['mean'], color=ORANGE, lw=1.5, label=f'{lb} ({B["n"]} Scans)')
    ax.set_ylabel('mittlere Distanz [m]')
    ax.set_title(f'Szene im Sensorkoordinatensystem um {abs(rot)}° verschoben '
                 f'(mittlere Abweichung nach Ausrichtung {err * 1000:.0f} mm)')
    ax.legend(loc='upper right', fontsize=9)

    ax = axs[1]
    ax.plot(ang, A['std'] * 100, color=BLUE, lw=1.5, label=la)
    ax.plot(ang, B['std'] * 100, color=ORANGE, lw=1.5, label=lb)
    ax.set_yscale('log')
    ax.set_ylabel('σ [cm] (log)')
    ax.set_title(f'σ je Strahlindex (sensorfest): Korrelation r = {c_sensor:.2f}')
    ax.legend(loc='upper right', fontsize=9)

    ax = axs[2]
    ax.plot(ang, np.roll(A['std'], k) * 100, color=BLUE, lw=1.5,
            label=f'{la}, um {k}° verschoben')
    ax.plot(ang, B['std'] * 100, color=ORANGE, lw=1.5, label=lb)
    ax.set_yscale('log')
    ax.set_ylabel('σ [cm] (log)')
    ax.set_title(f'σ nach Ausrichtung an der Umgebung: Korrelation r = {c_env:.2f}')
    ax.set_xlabel(f'Strahlwinkel [°] im Sensor-Frame von „{lb}“')
    ax.set_xlim(0, 359)
    ax.set_xticks(range(0, 361, 30))
    ax.legend(loc='upper right', fontsize=9)
    for a in axs:
        a.grid(True, alpha=0.3, which='both')
        a.spines[['top', 'right']].set_visible(False)
    fig.tight_layout()
    fig.savefig(args.out + '_ausrichtung.png', dpi=150)

    # sigma vs mean intensity and vs distance (both bags)
    fig, axs = plt.subplots(1, 2, figsize=(13, 5), sharey=True)
    for S, lab, col in [(A, la, BLUE), (B, lb, ORANGE)]:
        axs[0].scatter(S['intensity'], S['std'] * 100, s=12, color=col, alpha=0.6, label=lab)
        axs[1].scatter(S['mean'], S['std'] * 100, s=12, color=col, alpha=0.6, label=lab)
    axs[0].set_xlabel('mittlere Intensität des Strahls [-]')
    axs[0].set_ylabel('σ [cm] (log)')
    axs[0].set_title('Streuung vs. Signalstärke')
    axs[1].set_xlabel('mittlere Distanz [m]')
    axs[1].set_title('Streuung vs. Distanz')
    for a in axs:
        a.set_yscale('log')
        a.grid(True, alpha=0.3, which='both')
        a.legend(fontsize=9)
        a.spines[['top', 'right']].set_visible(False)
    fig.tight_layout()
    fig.savefig(args.out + '_sigma_intensitaet.png', dpi=150)

    print(f'{la}: {A["n"]} Scans, {lb}: {B["n"]} Scans')
    print(f'Verschiebung der Szene: {k} Strahlen (= {rot}°), '
          f'mittlere Abweichung danach {err * 1000:.1f} mm')
    print(f'Korrelation log(σ) sensorfest: {c_sensor:.2f}, umgebungsfest: {c_env:.2f}')
    print(f'Korrelation Ausfallrate sensorfest: {corr(1 - A["valid"], 1 - B["valid"]):.2f}, '
          f'umgebungsfest: {corr(np.roll(1 - A["valid"], k), 1 - B["valid"]):.2f}')
    for S, lab in [(A, la), (B, lb)]:
        print(f'{lab}: Korrelation log(σ)~Intensität {corr(np.log10(S["std"]), S["intensity"]):.2f}, '
              f'log(σ)~Distanz {corr(np.log10(S["std"]), S["mean"]):.2f}')
        for lo, hi in [(0, 2000), (2000, 3000), (3000, 3300), (3300, 1e9)]:
            s = (S['intensity'] >= lo) & (S['intensity'] < hi)
            print(f'   Intensität {lo:>5.0f}–{hi if hi < 1e9 else "∞":>5}: {s.sum():3d} Strahlen, '
                  f'Median σ {np.nanmedian(S["std"][s]) * 1000:5.1f} mm, '
                  f'Median Distanz {np.nanmedian(S["mean"][s]):.3f} m')
    print(f'\n-> {args.out}_ausrichtung.png\n-> {args.out}_sigma_intensitaet.png')


if __name__ == '__main__':
    main()
