import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nav_msgs.msg import Odometry


class OdomCov(Node):
    """Relay that stamps a realistic covariance onto wheel /odom.

    The Gazebo diff-drive plugin typically publishes /odom with a zero (or
    meaningless default) covariance. robot_localization reads the measurement
    covariance straight off the message, and a zero covariance makes the
    innovation matrix singular -> the Kalman gain blows up. This node copies
    /odom through to /odom_cov with sane diagonal covariances so the EKF has a
    well-conditioned measurement noise model.

    Values are tuned for a TurtleBot3 burger. Only the fused twist terms (vx,
    vyaw) really matter for the current EKF config; the rest are filled in for
    correctness in case more fields get fused later.
    """

    # Twist (body velocity) standard deviations.
    VX_STD = 0.02      # m/s   -- encoder-derived linear velocity, fairly good
    VY_STD = 0.02      # m/s   -- nonholonomic; not measured, kept tight
    VYAW_STD = 0.05    # rad/s -- wheel-derived yaw rate, slip-prone -> looser

    # Integrated-pose standard deviations (not fused today, set for sanity).
    X_STD = 0.05       # m
    Y_STD = 0.05       # m
    YAW_STD = 0.1      # rad

    # Large variance == "unobserved" for the out-of-plane DOFs.
    BIG = 1e3

    def __init__(self):
        super().__init__("odom_cov")
        self.sub = self.create_subscription(Odometry, "/odom", self.callback, 10)
        self.pub = self.create_publisher(Odometry, "/odom_cov", 10)

        # Row-major 6x6 covariance order: [x, y, z, roll, pitch, yaw].
        self.pose_cov = self._diag([
            self.X_STD ** 2, self.Y_STD ** 2, self.BIG,
            self.BIG, self.BIG, self.YAW_STD ** 2,
        ])
        # Row-major 6x6 covariance order: [vx, vy, vz, vroll, vpitch, vyaw].
        self.twist_cov = self._diag([
            self.VX_STD ** 2, self.VY_STD ** 2, self.BIG,
            self.BIG, self.BIG, self.VYAW_STD ** 2,
        ])

    @staticmethod
    def _diag(values):
        cov = [0.0] * 36
        for i, v in enumerate(values):
            cov[i * 6 + i] = float(v)
        return cov

    def callback(self, msg):
        msg.pose.covariance = self.pose_cov
        msg.twist.covariance = self.twist_cov
        self.pub.publish(msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = OdomCov()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()