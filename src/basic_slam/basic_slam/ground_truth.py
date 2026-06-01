import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry


class GroundTruth(Node):
    """Ground-truth pose relay for gz sim.

    gz publishes the true model poses on /world/<WORLD>/dynamic_pose/info. Bridge
    that to a ROS TFMessage:

        ros2 run ros_gz_bridge parameter_bridge \\
          /world/<WORLD>/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V

    then run this node with the bridged topic remapped onto 'gz_pose_tf'. It picks
    the transform whose child_frame_id matches `child_frame` (substring) and
    republishes it as nav_msgs/Odometry on /ground_truth, ready for evo.

    Note: some ros_gz versions deliver Pose_V -> TFMessage with EMPTY
    child_frame_id, so name matching fails. In dynamic_pose/info only the robot
    subtree moves and entry [0] is the robot's world pose, so we fall back to
    selecting by index when names are blank.

    Parameters:
      child_frame  (str)  substring identifying the robot link (default 'burger')
      pose_index   (int)  index to use when names are empty (default 0)
      frame_id     (str)  parent/world frame to stamp the odom with (default 'map')
    """

    def __init__(self):
        super().__init__("ground_truth")

        self.child_frame = self.declare_parameter("child_frame", "burger").value
        self.pose_index = self.declare_parameter("pose_index", 0).value
        self.frame_id = self.declare_parameter("frame_id", "map").value

        self.sub = self.create_subscription(TFMessage, "gz_pose_tf", self.callback, 50)
        self.pub = self.create_publisher(Odometry, "/ground_truth", 10)

        self._logged_frames = False
        self._warned_names = False

    def callback(self, msg):
        if not msg.transforms:
            return

        if not self._logged_frames:
            frames = sorted({t.child_frame_id for t in msg.transforms})
            self.get_logger().info(f'gz pose child frames seen: {frames}')
            self._logged_frames = True

        match = None
        for t in msg.transforms:
            if t.child_frame_id and self.child_frame in t.child_frame_id:
                match = t
                break

        if match is None:
            if not self._warned_names:
                self.get_logger().warn(
                    f'no named transform matched "{self.child_frame}"; '
                    f'falling back to pose_index={self.pose_index} '
                    '(gz bridge delivered empty frame names)')
                self._warned_names = True
            idx = self.pose_index if self.pose_index < len(msg.transforms) else 0
            match = msg.transforms[idx]

        stamp = match.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = match.child_frame_id
        odom.pose.pose.position.x = match.transform.translation.x
        odom.pose.pose.position.y = match.transform.translation.y
        odom.pose.pose.position.z = match.transform.translation.z
        odom.pose.pose.orientation = match.transform.rotation
        self.pub.publish(odom)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = GroundTruth()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()