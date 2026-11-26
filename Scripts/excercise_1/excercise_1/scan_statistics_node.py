"""
Mean, variance and standard deviation for each of the n beam directions.

Subscribes /scan (sensor_msgs/LaserScan, best effort like the LDS driver).
Writes a csv (one row per beam) every ``save_every`` scans and on shutdown:
  index, angle_deg, n_valid, valid_ratio, mean, variance, std, median, min, max
Invalid readings (0 / inf / outside [range_min, range_max]) are excluded and
counted via n_valid / valid_ratio.
"""

import os

from excercise_1.laser_core import BeamStatistics
from excercise_1.node_runner import run
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanStatistics(Node):

    def __init__(self):
        super().__init__('scan_statistics')
        self.declare_parameter('output_file', 'scan_statistics.csv')
        self.declare_parameter('save_every', 50)
        self.path = os.path.abspath(
            os.path.expanduser(self.get_parameter('output_file').value))
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self.save_every = max(1, self.get_parameter('save_every').value)
        self.stats = BeamStatistics()
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(f'Collecting /scan statistics -> {self.path}')

    def on_scan(self, msg):
        if len(self.stats) == 0:
            self.get_logger().info(
                f'first scan: {len(msg.ranges)} beams, angle_min={msg.angle_min:.4f}, '
                f'increment={msg.angle_increment:.6f} rad, '
                f'range=[{msg.range_min:.2f}, {msg.range_max:.2f}] m, frame={msg.header.frame_id}')
        self.stats.add_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                            msg.range_min, msg.range_max)
        if len(self.stats) % self.save_every == 0:
            self.save()

    def save(self):
        if len(self.stats) == 0:
            return
        s = self.stats.write_csv(self.path)
        self.get_logger().info(
            f'{len(self.stats)} scans -> {self.path} '
            f'(median std over beams: {_nanmedian(s["std"]) * 100:.2f} cm)')


def _nanmedian(a):
    import numpy as np
    return float(np.nanmedian(a))


def main(args=None):
    run(ScanStatistics, cleanup=ScanStatistics.save, args=args)


if __name__ == '__main__':
    main()
