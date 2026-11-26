"""
Own odometry for the TurtleBot3 computed directly from the wheel encoders.

Subscribes:
  /sensor_state  (turtlebot3_msgs/SensorState)  raw encoder ticks left/right
  /imu           (sensor_msgs/Imu)              only for the IMU variant

Publishes:
  /my_odom        (nav_msgs/Odometry)  pure wheel odometry (lecture formula)
  /my_odom_imu    (nav_msgs/Odometry)  encoder distance + IMU heading
                                       (= algorithm of the turtlebot3_node)
  /my_odom/path, /my_odom_imu/path (nav_msgs/Path) for RViz
  TF odom_frame -> base_frame from /my_odom   (only if publish_tf:=true)
"""

from excercise_1.node_runner import run
from excercise_1.odometry_core import (
    EncoderOdometry, normalize_angle, quaternion_from_yaw, yaw_from_quaternion)
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from turtlebot3_msgs.msg import SensorState


class MyOdometry(Node):

    def __init__(self):
        super().__init__('my_odometry')
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.160)
        self.declare_parameter('ticks_per_revolution', 4096)
        self.declare_parameter('method', 'approx')          # 'approx' | 'exact'
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('path_every_n', 5)  # add every n-th pose to the Path

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.path_every_n = max(1, self.get_parameter('path_every_n').value)

        self.variants = {}
        for name in ('my_odom', 'my_odom_imu'):
            self.variants[name] = {
                'odom': self._make_odometry(),
                'pub': self.create_publisher(Odometry, f'/{name}', 10),
                'path_pub': self.create_publisher(Path, f'/{name}/path', 10),
                'path': Path(),
            }

        self.imu_yaw = None
        self.prev_imu_yaw = None
        self.last_stamp = None
        self.count = 0
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(SensorState, '/sensor_state', self.on_sensor_state, 50)
        self.create_subscription(Imu, '/imu', self.on_imu, 50)
        self.get_logger().info(
            f'my_odometry started (method={self.get_parameter("method").value}, '
            f'publish_tf={self.publish_tf}, frames {self.odom_frame}->{self.base_frame})')

    def _make_odometry(self):
        return EncoderOdometry(
            wheel_radius=self.get_parameter('wheel_radius').value,
            wheel_separation=self.get_parameter('wheel_separation').value,
            ticks_per_rev=self.get_parameter('ticks_per_revolution').value,
            method=self.get_parameter('method').value)

    def reset(self):
        self.get_logger().warn('Time jumped backwards (bag restarted?) -> resetting odometry')
        for v in self.variants.values():
            v['odom'] = self._make_odometry()
            v['path'] = Path()
        self.prev_imu_yaw = None
        self.count = 0

    def on_imu(self, msg):
        q = msg.orientation
        self.imu_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def on_sensor_state(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is not None and stamp <= self.last_stamp:
            if self.last_stamp - stamp > 1.0:
                self.reset()  # big jump back: bag was restarted / looped
            else:
                # single late message (WiFi), e.g. in bag 09_56_59: its encoder
                # values are older than the last ones -> ignore it
                self.get_logger().warn(
                    f'out-of-order /sensor_state ({self.last_stamp - stamp:.3f} s old) skipped')
                return
        self.last_stamp = stamp

        dtheta_imu = None
        if self.imu_yaw is not None and self.prev_imu_yaw is not None:
            dtheta_imu = normalize_angle(self.imu_yaw - self.prev_imu_yaw)
        self.prev_imu_yaw = self.imu_yaw

        updated = self.variants['my_odom']['odom'].update(
            msg.left_encoder, msg.right_encoder, stamp)
        self.variants['my_odom_imu']['odom'].update(
            msg.left_encoder, msg.right_encoder, stamp, dtheta_override=dtheta_imu)
        if not updated:
            return

        self.count += 1
        for name, v in self.variants.items():
            odom_msg = self._to_msg(v['odom'], msg.header.stamp)
            v['pub'].publish(odom_msg)
            if self.count % self.path_every_n == 0:
                ps = PoseStamped()
                ps.header = odom_msg.header
                ps.pose = odom_msg.pose.pose
                v['path'].header = odom_msg.header
                v['path'].poses.append(ps)
                v['path_pub'].publish(v['path'])
            if name == 'my_odom' and self.tf_broadcaster is not None:
                self._send_tf(odom_msg)

    def _to_msg(self, odom, stamp):
        m = Odometry()
        m.header.stamp = stamp
        m.header.frame_id = self.odom_frame
        m.child_frame_id = self.base_frame
        m.pose.pose.position.x = odom.pose.x
        m.pose.pose.position.y = odom.pose.y
        qx, qy, qz, qw = quaternion_from_yaw(normalize_angle(odom.pose.theta))
        m.pose.pose.orientation.x = qx
        m.pose.pose.orientation.y = qy
        m.pose.pose.orientation.z = qz
        m.pose.pose.orientation.w = qw
        m.twist.twist.linear.x = odom.v
        m.twist.twist.angular.z = odom.w
        return m

    def _send_tf(self, odom_msg):
        t = TransformStamped()
        t.header = odom_msg.header
        t.child_frame_id = odom_msg.child_frame_id
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    run(MyOdometry, args=args)


if __name__ == '__main__':
    main()
