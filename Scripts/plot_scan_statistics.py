#!/usr/bin/env python3
"""
Plot the per-beam statistics written by the scan_statistics node.

  python3 Scripts/plot_scan_statistics.py Results/scan_statistics.csv

Creates <csv-name>_plot.png (cartesian), <csv-name>_polar.png and prints the
directions with significant deviations.
"""

import argparse
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


def load(path):
    with open(path) as f:
        first = f.readline()
    n_scans = int(first.split('=')[1]) if first.startswith('# scans=') else None
    skip = 1 if n_scans is not None else 0
    d = np.genfromtxt(path, delimiter=',', names=True, skip_header=skip)
    return d, n_scans


def main():
    plt.switch_backend('Agg')  # no window needed, only png files
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--image-dir', default=str(IMAGES), help='where the pngs go (default: Images/)')
    ap.add_argument('--factor', type=float, default=3.0,
                    help='std > factor * median(std) counts as significant (default 3)')
    args = ap.parse_args()
    d, n_scans = load(args.csv)
    os.makedirs(args.image_dir, exist_ok=True)
    base = os.path.join(args.image_dir, os.path.splitext(os.path.basename(args.csv))[0])

    ang = d['angle_deg']
    mean, std = d['mean'], d['std']
    has = np.isfinite(mean)
    med_std = np.nanmedian(std)
    sig = has & (std > args.factor * med_std)
    partial = has & (d['valid_ratio'] < 1.0)

    # ---------- cartesian overview ----------
    fig, axs = plt.subplots(3, 1, figsize=(13, 10), sharex=True,
                            gridspec_kw={'height_ratios': [2.2, 1.4, 1]})
    ax = axs[0]
    ax.fill_between(ang, d['min'], d['max'], color='0.85', label='Bereich min–max')
    ax.fill_between(ang, mean - std, mean + std, color='tab:blue', alpha=0.35,
                    label='Mittelwert ± σ')
    ax.plot(ang, mean, color='tab:blue', lw=1.2, label='Mittelwert')
    ax.plot(ang, d['median'], 'k--', lw=0.7, label='Median')
    ax.set_ylabel('Distanz [m]')
    ax.set_title(f'Laserscanner-Statistik pro Messrichtung ({n_scans} Scans, Roboter steht)')
    ax.legend(loc='upper right', fontsize=9, ncol=4)
    ax.grid(True, alpha=0.4)

    ax = axs[1]
    ax.bar(ang, std * 100, width=1.0, color=np.where(sig, 'tab:red', 'tab:blue'))
    ax.axhline(med_std * 100, color='k', lw=0.8, ls=':',
               label=f'Median σ = {med_std * 100:.2f} cm')
    ax.axhline(args.factor * med_std * 100, color='tab:red', lw=0.8, ls='--',
               label=f'{args.factor:g} × Median σ (rot = signifikant)')
    ax.set_yscale('log')
    ax.set_ylabel('σ [cm] (log)')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.4, which='both')

    ax = axs[2]
    ax.bar(ang, d['valid_ratio'] * 100, width=1.0, color='tab:gray')
    ax.set_ylabel('gültige Messungen [%]')
    ax.set_xlabel('Winkel [°] (0° = vorne, gegen den Uhrzeigersinn)')
    ax.set_xlim(-1, 360)
    ax.set_xticks(range(0, 361, 30))
    ax.grid(True, alpha=0.4)
    fig.tight_layout()
    fig.savefig(base + '_plot.png', dpi=150)

    # ---------- polar view (like on the exercise slides) ----------
    from matplotlib.colors import LogNorm
    fig = plt.figure(figsize=(9, 9))
    th = np.radians(ang)
    ax = fig.add_subplot(1, 1, 1, projection='polar')
    ax.fill_between(th, d['min'], d['max'], color='0.85', label='Bereich min–max')
    sc = ax.scatter(th[has], mean[has], c=std[has] * 100, s=14, cmap='plasma',
                    norm=LogNorm(vmin=max(np.nanmin(std[has]) * 100, 0.01),
                                 vmax=np.nanmax(std) * 100), zorder=3, label='Mittelwert')
    ax.plot(th[sig], mean[sig], 'o', color='red', ms=8, mfc='none', zorder=4,
            label=f'σ > {args.factor:g} × Median')
    ax.set_rlabel_position(250)
    ax.set_title(f'Mittlere Distanz [m] je Richtung, Farbe = σ ({n_scans} Scans)\n'
                 '0° = Fahrtrichtung, Winkel gegen den Uhrzeigersinn', pad=20)
    ax.legend(loc='upper left', bbox_to_anchor=(-0.08, 1.08), fontsize=9)
    fig.colorbar(sc, ax=ax, shrink=0.7, pad=0.08, label='σ [cm] (log)')
    fig.tight_layout()
    fig.savefig(base + '_polar.png', dpi=150)

    # ---------- std vs distance ----------
    fig, ax = plt.subplots(figsize=(7, 5))
    sc = ax.scatter(mean[has], std[has] * 100, c=d['valid_ratio'][has] * 100, s=12,
                    cmap='viridis')
    fig.colorbar(sc, label='gültige Messungen [%]')
    ax.set_yscale('log')
    ax.set_xlabel('Mittelwert [m]')
    ax.set_ylabel('σ [cm] (log)')
    ax.set_title('Streuung in Abhängigkeit der Distanz')
    ax.grid(True, alpha=0.4, which='both')
    fig.tight_layout()
    fig.savefig(base + '_std_vs_dist.png', dpi=150)

    # ---------- text summary ----------
    print(f'Scans: {n_scans}, Strahlen: {len(ang)}')
    print(f'Strahlen ohne jede gültige Messung: {np.sum(~has)} '
          f'(Winkel {_ranges(ang[~has])})')
    print(f'Strahlen mit Aussetzern (0 < gültig < 100 %): {np.sum(partial)}')
    print(f'σ: Median {med_std * 100:.2f} cm, Mittel {np.nanmean(std) * 100:.2f} cm, '
          f'Max {np.nanmax(std) * 100:.1f} cm')
    print(f'Signifikant (σ > {args.factor:g} × Median = {args.factor * med_std * 100:.2f} cm): '
          f'{np.sum(sig)} Strahlen')
    order = np.argsort(-np.nan_to_num(std))[:15]
    print('\nTop 15 Strahlen nach σ:')
    print('Winkel°  Mittel[m]  Median[m]  σ[cm]  min[m]  max[m]  gültig%')
    for i in order:
        print(f'{ang[i]:6.0f}  {mean[i]:9.3f}  {d["median"][i]:9.3f}  {std[i] * 100:5.1f}  '
              f'{d["min"][i]:6.3f}  {d["max"][i]:6.3f}  {d["valid_ratio"][i] * 100:6.1f}')
    print(f'\n-> {base}_plot.png\n-> {base}_polar.png\n-> {base}_std_vs_dist.png')


def _ranges(values):
    """[3,4,5,9] -> '3–5, 9'."""
    v = [int(round(x)) for x in values]
    if not v:
        return '-'
    out, start, prev = [], v[0], v[0]
    for x in v[1:] + [None]:
        if x is not None and x == prev + 1:
            prev = x
            continue
        out.append(f'{start}–{prev}' if prev != start else f'{start}')
        if x is not None:
            start = prev = x
    return ', '.join(out)


if __name__ == '__main__':
    main()
