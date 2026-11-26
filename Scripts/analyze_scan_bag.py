#!/usr/bin/env python3
"""
Offline variant of the scan_statistics + scan_sum nodes: reads /scan directly
from a bag (no replay, so no best-effort message can get lost) and writes the
same csv files using the same core functions (excercise_1/laser_core.py).

  source /opt/ros/jazzy/setup.bash
  python3 Scripts/analyze_scan_bag.py subset_5minSTILL            # -> Results/5minSTILL_*.csv

-> <prefix>_scan_statistics.csv, <prefix>_scan_sums.csv
   (default prefix: Results/<bag name without "subset_">)
Afterwards: plot_scan_statistics.py / plot_scan_histogram.py on these files.
"""

import argparse
import os
import sys
from pathlib import Path

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

# the core functions live in the ROS package Scripts/excercise_1/excercise_1/
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'excercise_1'))
from excercise_1.laser_core import BeamStatistics, scan_sum  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', help='bag folder or bag name in Messaufnahmen/')
    ap.add_argument('--prefix', help='output prefix (default: Results/<bag name>)')
    ap.add_argument('--topic', default='/scan')
    args = ap.parse_args()
    bag = bag_dir(args.bag)
    if args.prefix is None:
        args.prefix = str(RESULTS / bag.name.replace('subset_', ''))

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))

    os.makedirs(os.path.dirname(os.path.abspath(args.prefix)), exist_ok=True)
    stats = BeamStatistics()
    sums_path = args.prefix + '_scan_sums.csv'
    first = None
    with open(sums_path, 'w') as f:
        f.write('stamp,sum,n_valid\n')
        while reader.has_next():
            _, data, _ = reader.read_next()
            msg = deserialize_message(data, LaserScan)
            if first is None:
                first = msg
            stats.add_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                           msg.range_min, msg.range_max)
            total, n_valid = scan_sum(msg.ranges, msg.range_min, msg.range_max)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            f.write(f'{stamp:.9f},{total:.6f},{n_valid}\n')

    stats.write_csv(args.prefix + '_scan_statistics.csv')
    print(f'{len(stats)} scans, {len(first.ranges)} beams, '
          f'angle_min={first.angle_min:.4f}, increment={first.angle_increment:.6f} rad, '
          f'range=[{first.range_min:.3f}, {first.range_max:.3f}] m, '
          f'frame={first.header.frame_id}')
    print(f'-> {args.prefix}_scan_statistics.csv\n-> {sums_path}')


if __name__ == '__main__':
    main()
