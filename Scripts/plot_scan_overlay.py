#!/usr/bin/env python3
"""
x/y view of two static scans and overlay on a top-view photo of the test field.

  source /opt/ros/jazzy/setup.bash
  python3 Scripts/plot_scan_overlay.py subset_5minSTILL subset_5min_90d
  (defaults: --photo Images/Turtlebot3_TopView.jpeg --out Images/scan_xy)

Plotted is the mean range of every beam (the "mean scan" of the bag).
  <out>_sensor.png   both scans in the scanner frame (base_scan, x = forward),
                     left as recorded, right the second scan rotated back
  <out>_foto.png     both scans drawn on the photo (photo taken during bag 1)
  <out>_foto_sigma.png  scan 1 on the photo, coloured by the std. deviation per beam

Scan -> photo: similarity transform (scanner position in pixels, heading of the
scanner x axis in the image, scale in px/m at scanner height). The defaults were
fitted for Turtlebot3_TopView.jpeg by maximising the overlap of the scan points
with the plywood walls; --fit repeats this fit starting from the given values.
Scan 2 is first registered onto scan 1 (rotation + translation, nearest
neighbour), so it is drawn with the pose the robot had after turning.
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, FancyArrowPatch
from rclpy.serialization import deserialize_message
from scipy.optimize import minimize
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan

# Repo-Ordnerstruktur: Scripts/ (dieses Skript), Messaufnahmen/, Results/, Images/
REPO = Path(__file__).resolve().parent.parent
BAGS, RESULTS, IMAGES = REPO / 'Messaufnahmen', REPO / 'Results', REPO / 'Images'


def bag_dir(p):
    """Bag folder as given, or by name from Messaufnahmen/ (e.g. 'subset_5minSTILL')."""
    p = Path(p)
    return p if p.exists() else BAGS / p

STRONG, WEAK = '#257de0', '#f47230'


def mean_scan(bag):
    """Mean range per beam -> (360, 2) points in the scanner frame [m]."""
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
    rows, m = [], None
    while r.has_next():
        m = deserialize_message(r.read_next()[1], LaserScan)
        rows.append(np.asarray(m.ranges, float))
    x = np.vstack(rows)
    x[~(np.isfinite(x) & (x >= m.range_min) & (x <= m.range_max))] = np.nan
    with np.errstate(all='ignore'):
        rng = np.nanmean(x, 0)
        std = np.nanstd(x, 0, ddof=1)
    th = m.angle_min + np.arange(len(rng)) * m.angle_increment
    return np.c_[rng * np.cos(th), rng * np.sin(th)], len(rows), std


def rot(p, a, t=(0.0, 0.0)):
    c, s = np.cos(a), np.sin(a)
    return np.c_[c * p[:, 0] - s * p[:, 1] + t[0], s * p[:, 0] + c * p[:, 1] + t[1]]


def register(src, dst, a0):
    """Rotation a and translation t with dst ~ rot(src, a, t)."""
    ok = np.all(np.isfinite(dst), 1)
    d = dst[ok]
    dense = np.vstack([d + f * (np.roll(d, -1, 0) - d) for f in np.linspace(0, 1, 5)])
    tree = cKDTree(dense)
    s = src[np.all(np.isfinite(src), 1)]

    def cost(q):
        dist, _ = tree.query(rot(s, q[0], q[1:]))
        return np.mean(np.minimum(dist, 0.05) ** 2)

    best = min((minimize(cost, (a0 + da, 0, 0), method='Nelder-Mead')
                for da in np.radians([-4, 0, 4])), key=lambda r: r.fun)
    return best.x[0], best.x[1:], np.sqrt(best.fun)


def to_px(p, cx, cy, heading, scale):
    """Scanner frame [m] -> image pixels (image y axis points down)."""
    q = rot(p, heading) * scale
    return np.c_[cx + q[:, 0], cy - q[:, 1]]


def fit_photo(img, pts, start):
    """Refine (cx, cy, heading, scale): scan points should lie on the plywood walls."""
    import cv2
    hsv = cv2.cvtColor((img * 255).astype(np.uint8) if img.max() <= 1 else img,
                       cv2.COLOR_RGB2HSV).astype(int)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    wall = ((v > 150) & (s > 25) & (s < 110) & (h >= 8) & (h <= 30)) | ((v > 185) & (s <= 25))
    dist = cv2.distanceTransform((~wall).astype(np.uint8), cv2.DIST_L2, 5)
    from scipy.ndimage import map_coordinates
    ok = np.all(np.isfinite(pts), 1)

    def cost(p):
        uv = to_px(pts[ok], p[0], p[1], p[2], p[3])
        d = map_coordinates(dist, [uv[:, 1], uv[:, 0]], order=1, mode='constant', cval=200)
        return np.mean(np.minimum(d, 25) ** 2)

    return minimize(cost, start, method='Nelder-Mead', options={'maxiter': 4000}).x


def robot(ax, xy, heading, length, color, label, lw=2.5, alpha=1.0):
    ax.add_patch(FancyArrowPatch(xy, (xy[0] + length * np.cos(heading),
                                      xy[1] + length * np.sin(heading)),
                                 arrowstyle='-|>', mutation_scale=18, color=color,
                                 lw=lw, alpha=alpha, label=label, zorder=5))


def main():
    plt.switch_backend('Agg')
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_a', help='bag recorded while the photo was taken')
    ap.add_argument('bag_b', help='bag with the rotated robot')
    ap.add_argument('--labels', nargs=2,
                    default=['Ausgangslage (5minSTILL)', 'um 90° gedreht (5min_90d)'])
    ap.add_argument('--photo', default=str(IMAGES / 'Turtlebot3_TopView.jpeg'))
    ap.add_argument('--out', default=str(IMAGES / 'scan_xy'), help='output prefix for the pngs')
    ap.add_argument('--pose', nargs=4, type=float, default=[982.2, 610.8, 19.58, 590.7],
                    metavar=('CX', 'CY', 'HEADING_DEG', 'PX_PER_M'),
                    help='scanner pose in the photo (default: fitted for Turtlebot3_TopView)')
    ap.add_argument('--fit', action='store_true', help='refine --pose automatically')
    ap.add_argument('--crop', nargs=4, type=int, default=[330, 250, 1560, 1100],
                    metavar=('X0', 'Y0', 'X1', 'Y1'))
    args = ap.parse_args()
    la, lb = args.labels

    A, na, std_a = mean_scan(bag_dir(args.bag_a))
    B, nb, _ = mean_scan(bag_dir(args.bag_b))
    a, t, rms = register(B, A, np.radians(-90))
    B_in_A = rot(B, a, t)

    # ---------- x/y in the scanner frame ----------
    fig, axs = plt.subplots(1, 2, figsize=(14, 7), sharex=True, sharey=True)
    for ax, pb, title in [
            (axs[0], B, 'wie aufgenommen (Scanner-Frame base_scan)'),
            (axs[1], B_in_A, f'Scan 2 um {np.degrees(a):.1f}° zurückgedreht '
                             f'(Restabweichung {rms * 1000:.0f} mm)')]:
        ax.plot(pb[:, 0], pb[:, 1], 'o', ms=4, color=WEAK, alpha=0.35, mec='none',
                label=f'{lb}, {nb} Scans')
        ax.plot(A[:, 0], A[:, 1], 'o', ms=3, color=STRONG, mec='none',
                label=f'{la}, {na} Scans')
        ax.add_patch(Circle((0, 0), 0.07, fc='0.85', ec='0.4', zorder=4))
        robot(ax, (0, 0), 0.0, 0.2, STRONG, None)
        if ax is axs[1]:
            robot(ax, (t[0], t[1]), a, 0.2, WEAK, None, alpha=0.6)
        ax.set_title(title, fontsize=11)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('x [m] (Fahrtrichtung)')
        ax.spines[['top', 'right']].set_visible(False)
    axs[0].set_ylabel('y [m] (links)')
    h, lab = axs[0].get_legend_handles_labels()
    fig.legend(h[::-1], lab[::-1], loc='lower center', ncol=2, fontsize=10,
               markerscale=2, frameon=False)
    fig.suptitle('Mittlerer Laserscan je Strahl in x/y-Darstellung '
                 '(Pfeil = x-Achse des Scanners)', fontsize=13)
    fig.tight_layout(rect=(0, 0.05, 1, 0.96))
    fig.savefig(args.out + '_sensor.png', dpi=150, bbox_inches='tight')

    # ---------- overlay on the photo ----------
    img = plt.imread(args.photo)
    cx, cy, hd, sc = args.pose
    if args.fit:
        cx, cy, h, sc = fit_photo(img, A, (cx, cy, np.radians(hd), sc))
        hd = np.degrees(h)
    hr = np.radians(hd)
    ua, ub = to_px(A, cx, cy, hr, sc), to_px(B_in_A, cx, cy, hr, sc)
    pa, pb = to_px(np.zeros((1, 2)), cx, cy, hr, sc)[0], to_px(t[None], cx, cy, hr, sc)[0]

    x0, y0, x1, y1 = args.crop
    fig, ax = plt.subplots(figsize=(12, 12 * (y1 - y0) / (x1 - x0)))
    ax.imshow(img)
    ax.plot(ub[:, 0], ub[:, 1], 'o', ms=7, color=WEAK, alpha=0.45, mec='none',
            label=f'{lb} (mit gedrehter Pose eingetragen)')
    ax.plot(ua[:, 0], ua[:, 1], 'o', ms=3.5, color=STRONG, mec='white', mew=0.4,
            label=f'{la} (Foto während dieser Aufnahme)')
    # image y axis points down -> heading in image coordinates is mirrored
    robot(ax, tuple(pa), -hr, 0.25 * sc, STRONG, None)
    robot(ax, tuple(pb), -(hr + a), 0.25 * sc, WEAK, None, alpha=0.7)
    bx, by = x0 + 40, y1 - 40   # 0.5 m scale bar
    ax.plot([bx, bx + 0.5 * sc], [by, by], color='white', lw=4, solid_capstyle='butt')
    ax.text(bx + 0.25 * sc, by - 12, '0,5 m (Scannerhöhe)', color='white', ha='center',
            fontsize=10, fontweight='bold')
    ax.set_xlim(x0, x1)
    ax.set_ylim(y1, y0)
    ax.set_axis_off()
    handles, labels = ax.get_legend_handles_labels()
    handles = handles[:2] + [Line2D([], [], color=STRONG, lw=2.5, marker='>', ms=7),
                             Line2D([], [], color=WEAK, lw=2.5, marker='>', ms=7, alpha=0.7)]
    fig.legend(handles, labels[:2] + ['Fahrtrichtung Ausgangslage',
                                      'Fahrtrichtung nach der Drehung'],
               loc='lower center', ncol=2, fontsize=10, markerscale=1.3, frameon=False)
    ax.set_title('Mittlerer Laserscan über dem Foto des Testfelds', fontsize=13)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(args.out + '_foto.png', dpi=150)

    # ---------- photo, scan 1 coloured by the standard deviation (point 4) ----------
    from matplotlib.colors import LogNorm
    fig, ax = plt.subplots(figsize=(12, 12 * (y1 - y0) / (x1 - x0)))
    ax.imshow(img)
    ok = np.isfinite(std_a)
    sca = ax.scatter(ua[ok, 0], ua[ok, 1], c=std_a[ok] * 1000, s=22, cmap='plasma',
                     norm=LogNorm(vmin=0.3, vmax=40), edgecolors='white', linewidths=0.3)
    robot(ax, tuple(pa), -hr, 0.25 * sc, 'white', None)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y1, y0)
    ax.set_axis_off()
    fig.colorbar(sca, ax=ax, shrink=0.8, pad=0.01, label='Standardabweichung σ [mm] (log)')
    ax.set_title(f'{la}: Streuung je Strahl auf dem Foto', fontsize=13)
    fig.tight_layout()
    fig.savefig(args.out + '_foto_sigma.png', dpi=150, bbox_inches='tight')

    print(f'{la}: {na} Scans, {lb}: {nb} Scans')
    print(f'Scan 2 -> Scan 1: Drehung {np.degrees(a):.2f}°, Versatz '
          f'({t[0] * 1000:.0f}, {t[1] * 1000:.0f}) mm, Restabweichung {rms * 1000:.1f} mm')
    print(f'Pose im Foto: Scanner bei ({cx:.1f}, {cy:.1f}) px, x-Achse {hd:.2f}° '
          f'(gegen den Uhrzeigersinn ab Bild-rechts), Maßstab {sc:.1f} px/m')
    print(f'-> {args.out}_sensor.png\n-> {args.out}_foto.png\n-> {args.out}_foto_sigma.png')


if __name__ == '__main__':
    main()
