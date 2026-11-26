"""
Writes the trajectories of several odometry topics into ONE csv file.

Default topics:
  /odom         odometry of the turtlebot3_node
  /my_odom      own wheel odometry          (my_odometry node)
  /my_odom_imu  own encoder + IMU odometry  (my_odometry node)

CSV columns: topic, stamp [s], x [m], y [m], yaw [rad]
(plot with Scripts/plot_trajectories.py)
"""

import os

from excercise_1.node_runner import run
from excercise_1.odometry_core import yaw_from_quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node


class TrajectoryLogger(Node):

    def __init__(self):
        super().__init__('trajectory_logger')
        self.declare_parameter('topics', ['/odom', '/my_odom', '/my_odom_imu'])
        self.declare_parameter('output_file', 'trajectories.csv')
        topics = self.get_parameter('topics').value
        path = os.path.abspath(os.path.expanduser(self.get_parameter('output_file').value))
        os.makedirs(os.path.dirname(path), exist_ok=True)

        self.file = open(path, 'w', buffering=1)  # line buffered -> nothing lost on Ctrl+C
        self.file.write('topic,stamp,x,y,yaw\n')
        self.counts = {t: 0 for t in topics}
        for topic in topics:
            self.create_subscription(
                Odometry, topic, lambda msg, t=topic: self.on_odom(t, msg), 50)
        self.create_timer(5.0, self.report)
        self.get_logger().info(f'Logging {topics} -> {path}')

    def on_odom(self, topic, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        # %.9g instead of %f: the recorded /odom contains values like -6.7e248
        self.file.write(f'{topic},{stamp:.9f},{p.x:.9g},{p.y:.9g},{yaw:.9g}\n')
        self.counts[topic] += 1

    def report(self):
        self.get_logger().info('poses written: ' + ', '.join(
            f'{t}={n}' for t, n in self.counts.items()))

    def close(self):
        if not self.file.closed:
            self.file.close()
            self.get_logger().info('file closed: ' + ', '.join(
                f'{t}={n}' for t, n in self.counts.items()))


def main(args=None):
    run(TrajectoryLogger, cleanup=TrajectoryLogger.close, args=args)


if __name__ == '__main__':
    main()
