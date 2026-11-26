"""
Common main() for all nodes of this package.

On Ctrl+C the terminal sends SIGINT to every process AND ros2 launch forwards
another SIGINT to its children. Without ignoring the second one, it can
interrupt the cleanup (e.g. writing the result csv) halfway.
"""

import signal

import rclpy
from rclpy.executors import ExternalShutdownException


def run(node_cls, cleanup=None, args=None):
    rclpy.init(args=args)
    node = node_cls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if cleanup is not None:
            cleanup(node)
        node.destroy_node()
        rclpy.try_shutdown()
