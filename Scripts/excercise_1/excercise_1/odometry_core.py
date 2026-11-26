"""
Differential-drive odometry for the TurtleBot3 Burger (ROS-independent core).

Used by the ROS node ``my_odometry``.

Lecture formulas (Delta s_l, Delta s_r = distance travelled by the left/right wheel):

    Delta s     = (Delta s_r + Delta s_l) / 2
    Delta theta = (Delta s_r - Delta s_l) / b

  Approximation (mean heading of the step):
    x'     = x + Delta s * cos(theta + Delta theta / 2)
    y'     = y + Delta s * sin(theta + Delta theta / 2)
    theta' = theta + Delta theta

  Exact (circular arc with radius R = Delta s / Delta theta):
    x'     = x + R * (sin(theta + Delta theta) - sin(theta))
    y'     = y - R * (cos(theta + Delta theta) - cos(theta))
    theta' = theta + Delta theta
"""

from dataclasses import dataclass
import math

# TurtleBot3 Burger (robotis e-Manual / turtlebot3_node parameters)
WHEEL_RADIUS = 0.033          # [m]
WHEEL_SEPARATION = 0.160      # [m]
TICKS_PER_REVOLUTION = 4096   # Dynamixel XL430 encoder ticks per wheel revolution


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # continuous (not wrapped), use normalize_angle() for output


def normalize_angle(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw):
    """Return (x, y, z, w) of a rotation about the z axis."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def integrate_approx(pose, ds, dtheta):
    th = pose.theta + dtheta / 2.0
    return Pose2D(pose.x + ds * math.cos(th),
                  pose.y + ds * math.sin(th),
                  pose.theta + dtheta)


def integrate_exact(pose, ds, dtheta):
    if abs(dtheta) < 1e-9:  # straight line -> radius infinite
        return Pose2D(pose.x + ds * math.cos(pose.theta),
                      pose.y + ds * math.sin(pose.theta),
                      pose.theta)
    r = ds / dtheta
    th_new = pose.theta + dtheta
    return Pose2D(pose.x + r * (math.sin(th_new) - math.sin(pose.theta)),
                  pose.y - r * (math.cos(th_new) - math.cos(pose.theta)),
                  th_new)


class EncoderOdometry:
    """
    Integrates wheel encoder ticks into a 2D pose.

    If ``dtheta_override`` is given to :meth:`update`, the heading change is
    taken from it (e.g. from the IMU) instead of from the wheel difference.
    This is what the original turtlebot3_node does (encoders for distance,
    IMU for heading).
    """

    def __init__(self, wheel_radius=WHEEL_RADIUS, wheel_separation=WHEEL_SEPARATION,
                 ticks_per_rev=TICKS_PER_REVOLUTION, method='approx'):
        if method not in ('approx', 'exact'):
            raise ValueError("method must be 'approx' or 'exact'")
        self.meters_per_tick = 2.0 * math.pi * wheel_radius / ticks_per_rev
        self.b = wheel_separation
        self._integrate = integrate_approx if method == 'approx' else integrate_exact
        self.pose = Pose2D()
        self.v = 0.0   # [m/s]
        self.w = 0.0   # [rad/s]
        self.distance = 0.0  # travelled path length [m]
        self._last = None    # (left_ticks, right_ticks, stamp)

    def update(self, left_ticks, right_ticks, stamp, dtheta_override=None):
        """Feed one encoder reading. Returns True if the pose was updated."""
        if self._last is None:
            self._last = (left_ticks, right_ticks, stamp)
            return False
        l0, r0, t0 = self._last
        self._last = (left_ticks, right_ticks, stamp)
        ds_l = (left_ticks - l0) * self.meters_per_tick
        ds_r = (right_ticks - r0) * self.meters_per_tick
        ds = (ds_r + ds_l) / 2.0
        dtheta = (ds_r - ds_l) / self.b if dtheta_override is None else dtheta_override
        self.pose = self._integrate(self.pose, ds, dtheta)
        self.distance += abs(ds)
        dt = stamp - t0
        if dt > 1e-6:
            self.v = ds / dt
            self.w = dtheta / dt
        return True
