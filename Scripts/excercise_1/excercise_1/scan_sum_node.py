"""
Sums up all distance values of every single laser scan.

Subscribes /scan and appends one line per scan to a csv:
  stamp [s], sum [m], n_valid
The histogram is created afterwards with Scripts/plot_scan_histogram.py.
"""

import os

from excercise_1.laser_core import scan_sum
from excercise_1.node_runner import run
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanSum(Node):

    def __init__(self):
        super().__init__('scan_sum')
        self.declare_parameter('output_file', 'scan_sums.csv')
        path = os.path.abspath(os.path.expanduser(self.get_parameter('output_file').value))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.file = open(path, 'w', buffering=1)
        self.file.write('stamp,sum,n_valid\n')
        self.count = 0
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(f'Writing scan sums -> {path}')

    def on_scan(self, msg):
        total, n_valid = scan_sum(msg.ranges, msg.range_min, msg.range_max)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.file.write(f'{stamp:.9f},{total:.6f},{n_valid}\n')
        self.count += 1
        if self.count % 50 == 0:
            self.get_logger().info(
                f'{self.count} scans, last sum = {total:.2f} m ({n_valid} valid)')

    def close(self):
        if not self.file.closed:
            self.file.close()
            self.get_logger().info(f'file closed ({self.count} scans)')


def main(args=None):
    run(ScanSum, cleanup=ScanSum.close, args=args)


if __name__ == '__main__':
    main()
