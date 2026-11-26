"""
Republishes recorded TF but drops selected transforms.

In our bags the turtlebot3_node odometry is broken (x ~ -6.7e248, constant
yaw), and so are the TF odom->base_footprint and cartographer's map->odom.
Huge values like these make RViz unusable. The launch files therefore play
the bag with /tf remapped to /tf_bag; this node forwards everything else
(e.g. base_link->wheel_*_link) to /tf.
"""

from excercise_1.node_runner import run
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage


class TfFilter(Node):

    def __init__(self):
        super().__init__('tf_filter')
        self.declare_parameter('input_topic', '/tf_bag')
        self.declare_parameter('drop_child_frames', ['odom', 'base_footprint'])
        self.drop = set(self.get_parameter('drop_child_frames').value)
        qos = QoSProfile(depth=100)
        self.pub = self.create_publisher(TFMessage, '/tf', qos)
        self.create_subscription(
            TFMessage, self.get_parameter('input_topic').value, self.on_tf, qos)
        self.get_logger().info(f'Forwarding TF, dropping child frames {sorted(self.drop)}')

    def on_tf(self, msg):
        keep = [t for t in msg.transforms if t.child_frame_id not in self.drop]
        if keep:
            self.pub.publish(TFMessage(transforms=keep))


def main(args=None):
    run(TfFilter, args=args)


if __name__ == '__main__':
    main()
