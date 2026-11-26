#!/usr/bin/env python3
"""
Histogram of the per-scan distance sums written by the scan_sum node.

  python3 Scripts/plot_scan_histogram.py Results/scan_sums.csv [--bin-width 1.0]

Bin width (--bin-width):
  'fd'     Freedman-Diaconis  h = 2 * IQR * n^(-1/3)   (default, robust to outliers)
  'scott'  Scott              h = 3.49 * sigma * n^(-1/3)
  'sturges'                   k = log2(n) + 1 bins
  <number> fixed width in metres
The script prints all three so the choice can be justified in the report.
"""

import argparse
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


def bin_widths(x):
    n = len(x)
    iqr = np.subtract(*np.percentile(x, [75, 25]))
    return {
        'fd': 2 * iqr * n ** (-1 / 3),
        'scott': 3.49 * np.std(x, ddof=1) * n ** (-1 / 3),
        'sturges': (x.max() - x.min()) / (math.log2(n) + 1),
    }


def edges(x, width):
    lo = math.floor(x.min() / width) * width
    hi = math.ceil(x.max() / width) * width + width / 2
    return np.arange(lo, hi + width / 2, width)


def main():
    plt.switch_backend('Agg')  # no window needed, only png files
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv')
    ap.add_argument('--bin-width', default='fd')
    ap.add_argument('--image-dir', default=str(IMAGES), help='where the png goes (default: Images/)')
    args = ap.parse_args()

    d = np.genfromtxt(args.csv, delimiter=',', names=True)
    os.makedirs(args.image_dir, exist_ok=True)
    base = os.path.join(args.image_dir, os.path.splitext(os.path.basename(args.csv))[0])
    s, nv, t = d['sum'], d['n_valid'], d['stamp'] - d['stamp'][0]
    per_beam = s / nv

    widths = bin_widths(s)
    if args.bin_width in widths:
        w, w_name = widths[args.bin_width], args.bin_width
    else:
        w, w_name = float(args.bin_width), 'fest'

    mu, sd = s.mean(), s.std(ddof=1)
    slope, icpt = np.polyfit(nv, s, 1)
    r = np.corrcoef(nv, s)[0, 1]

    fig, axs = plt.subplots(2, 2, figsize=(14, 9))
    ax = axs[0, 0]
    # stacked by number of valid beams: every missing beam shifts the sum by ~0.7 m,
    # so the histogram is a mix of one cluster per dropout count
    groups = [(360, '360 gültig'), (359, '359'), (358, '358'), (357, '357'), (0, '≤ 356')]
    ramp = ['#0f4a96', '#1b64be', '#338ced', '#6aabf1', '#9dc8f7']  # dark = no dropout
    parts, labels, colors = [], [], []
    for (n, lab), col in zip(groups, ramp):
        sel = (nv == n) if n else (nv <= 356)
        if sel.any():
            parts.append(s[sel])
            pct = sel.mean() * 100
            labels.append(f'{lab} ({pct:.0f} %)' if pct >= 1 else f'{lab} ({sel.sum()} Scans)')
            colors.append(col)
    ax.hist(parts, bins=edges(s, w), stacked=True, color=colors, edgecolor='white',
            linewidth=0.5, label=labels)
    xs = np.linspace(s.min() - sd, s.max() + sd, 300)
    ax.plot(xs, len(s) * w / (sd * math.sqrt(2 * math.pi)) * np.exp(-0.5 * ((xs - mu) / sd) ** 2),
            'k--', lw=1, label='Normalverteilung (μ, σ)')
    ax.axvline(mu, color='tab:red', lw=1.2, label=f'μ = {mu:.1f} m')
    ax.axvspan(mu - sd, mu + sd, color='tab:red', alpha=0.1, label=f'± σ = {sd:.1f} m')
    ax.set_title(f'Histogramm der Distanzsummen je Scan (n = {len(s)}, '
                 f'Klassenbreite {w:.2f} m [{w_name}])')
    ax.set_xlabel('Summe aller gültigen Distanzen eines Scans [m]')
    ax.set_ylabel('Anzahl Scans')
    ax.legend(fontsize=8, title='gültige Strahlen im Scan', title_fontsize=8)
    ax.grid(True, alpha=0.4)

    # time course on ONE axis: raw sum and the sum normalised to 360 beams
    # (sum / n_valid * 360 removes the dropout jumps, a slow drift becomes visible)
    ax = axs[0, 1]
    norm = per_beam * 360
    win = max(1, int(round(10 / np.median(np.diff(t)))))  # ~10 s moving average
    roll = np.convolve(norm, np.ones(win) / win, mode='valid')
    t_roll = t[win // 2: win // 2 + len(roll)]
    ax.plot(t, s, '.', color='#9dc8f7', ms=3, label='Summe (roh)')
    ax.plot(t, norm, '.', color='#f9b48f', ms=2, label='Summe auf 360 Strahlen normiert')
    ax.plot(t_roll, roll, color='#f47230', lw=2, label='normiert, gleitendes Mittel 10 s')
    ax.set_xlabel('t [s]')
    ax.set_ylabel('Summe [m]')
    gaps = np.where(np.diff(t) > 1.0)[0]
    for g in gaps:
        ax.axvspan(t[g], t[g + 1], color='k', alpha=0.08)
    ax.set_title('Zeitverlauf' + (' (grau = Pause zwischen den Bags)' if len(gaps) else ''))
    ax.legend(fontsize=8, loc='lower right')
    ax.grid(True, alpha=0.4)

    ax = axs[1, 0]
    ax.scatter(nv, s, s=10, alpha=0.6)
    xx = np.array([nv.min(), nv.max()])
    ax.plot(xx, slope * xx + icpt, 'r-', lw=1,
            label=f'Regression: {slope:.2f} m pro gültigem Strahl, r = {r:.2f}')
    ax.set_xlabel('Anzahl gültiger Strahlen im Scan')
    ax.set_ylabel('Summe [m]')
    ax.set_title('Summe vs. Anzahl gültiger Messungen')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.4)

    ax = axs[1, 1]
    wb = bin_widths(per_beam)['fd']
    ax.hist(per_beam, bins=edges(per_beam, wb), color='tab:green', edgecolor='white')
    ax.set_title(f'Summe / Anzahl gültiger Strahlen (μ = {per_beam.mean():.3f} m, '
                 f'σ = {per_beam.std(ddof=1) * 100:.1f} cm)')
    ax.set_xlabel('mittlere Distanz pro gültigem Strahl [m]')
    ax.set_ylabel('Anzahl Scans')
    ax.grid(True, alpha=0.4)

    fig.tight_layout()
    fig.savefig(base + '_histogram.png', dpi=150)

    print(f'Scans: {len(s)}')
    print(f'Summe: min {s.min():.2f}  max {s.max():.2f}  μ {mu:.2f}  median {np.median(s):.2f}  '
          f'σ {sd:.2f} m  (σ/μ = {sd / mu * 100:.1f} %)')
    print(f'Gültige Strahlen: {int(nv.min())}–{int(nv.max())} von 360 '
          f'(μ {nv.mean():.1f}, σ {nv.std(ddof=1):.1f})')
    print('Klassenbreiten-Regeln: ' + ', '.join(f'{k} = {v:.2f} m' for k, v in widths.items()))
    print(f'Gewählt: {w:.2f} m ({w_name}) -> {len(edges(s, w)) - 1} Klassen')
    print(f'Regression Summe ~ gültige Strahlen: {slope:.2f} m/Strahl, r = {r:.2f} '
          f'(r² = {r * r:.2f} der Varianz durch Aussetzer erklärt)')
    print(f'Summe / gültige Strahlen: μ {per_beam.mean():.3f} m, '
          f'σ {per_beam.std(ddof=1) * 100:.2f} cm')
    for n in range(int(nv.min()), 361):
        sel = nv == n
        if sel.sum() > 1:
            print(f'  {n} gültige Strahlen: {sel.sum():4d} Scans ({sel.mean() * 100:4.1f} %), '
                  f'Summe μ {s[sel].mean():.2f} m, σ {s[sel].std(ddof=1):.2f} m')
    drift = roll[-1] - roll[0]
    print(f'Normierte Summe (10-s-Mittel): Anfang {roll[0]:.2f} m, Ende {roll[-1]:.2f} m, '
          f'max {roll.max():.2f} m bei t = {t_roll[np.argmax(roll)]:.0f} s '
          f'-> Drift {drift:+.2f} m = {drift / 360 * 1000:+.2f} mm pro Strahl')
    print(f'\n-> {base}_histogram.png')


if __name__ == '__main__':
    main()
