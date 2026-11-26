"""
Laser scan statistics (ROS-independent core).

Used by the ROS nodes ``scan_statistics`` and ``scan_sum``.

Invalid readings: the TurtleBot3 LDS reports 0.0 (sometimes inf/nan) when no
echo was received (out of range, black / glossy surface, glass ...). These are
excluded from the statistics and counted separately.
"""

import math
import os

import numpy as np


def valid_ranges(ranges, range_min, range_max):
    """Return ranges as float array with invalid readings replaced by NaN."""
    r = np.asarray(ranges, dtype=float)
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    return np.where(ok, r, np.nan)


class BeamStatistics:
    """Collects scans and computes mean / variance / std per beam direction."""

    def __init__(self):
        self.scans = []
        self.angle_min = None
        self.angle_increment = None

    def add_scan(self, ranges, angle_min, angle_increment, range_min, range_max):
        if self.angle_min is None:
            self.angle_min = angle_min
            self.angle_increment = angle_increment
        self.scans.append(valid_ranges(ranges, range_min, range_max))

    def __len__(self):
        return len(self.scans)

    def compute(self):
        """Return a dict of per-beam arrays (length n = beams per scan)."""
        data = np.vstack(self.scans)                 # shape (n_scans, n_beams)
        n_valid = np.sum(np.isfinite(data), axis=0)
        with np.errstate(all='ignore'), _quiet_nan_warnings():
            mean = np.nanmean(data, axis=0)
            var = np.nanvar(data, axis=0, ddof=1)    # sample variance
            median = np.nanmedian(data, axis=0)
            rmin = np.nanmin(data, axis=0)
            rmax = np.nanmax(data, axis=0)
        n_beams = data.shape[1]
        idx = np.arange(n_beams)
        return {
            'index': idx,
            'angle_deg': np.degrees(self.angle_min + idx * self.angle_increment),
            'n_valid': n_valid,
            'valid_ratio': n_valid / data.shape[0],
            'mean': mean,
            'variance': var,
            'std': np.sqrt(var),
            'median': median,
            'min': rmin,
            'max': rmax,
        }

    def write_csv(self, path):
        s = self.compute()
        cols = ['index', 'angle_deg', 'n_valid', 'valid_ratio',
                'mean', 'variance', 'std', 'median', 'min', 'max']
        tmp = path + '.tmp'  # write + rename: never leaves a half written file
        with open(tmp, 'w') as f:
            f.write(f'# scans={len(self.scans)}\n')
            f.write(','.join(cols) + '\n')
            for i in range(len(s['index'])):
                f.write(','.join(_fmt(s[c][i]) for c in cols) + '\n')
        os.replace(tmp, path)
        return s


def scan_sum(ranges, range_min, range_max):
    """Sum of all valid distance values of one scan -> (sum, n_valid)."""
    r = valid_ranges(ranges, range_min, range_max)
    ok = np.isfinite(r)
    return float(np.sum(r[ok])), int(np.sum(ok))


def _fmt(v):
    if isinstance(v, (int, np.integer)):
        return str(int(v))
    return 'nan' if not math.isfinite(v) else f'{v:.6f}'


class _quiet_nan_warnings:
    """Suppress 'All-NaN slice' / 'Degrees of freedom' RuntimeWarnings."""

    def __enter__(self):
        import warnings
        self._cm = warnings.catch_warnings()
        self._cm.__enter__()
        warnings.simplefilter('ignore', category=RuntimeWarning)

    def __exit__(self, *exc):
        return self._cm.__exit__(*exc)
