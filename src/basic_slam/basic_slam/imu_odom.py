import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from messages.msg import PosYaw

import math

class ImuNode(Node):
    
    def __init__(self):
        super().__init__("imu_node")
        self.imu_subscription = self.create_subscription(
            Imu,
            "/imu",
            self.callback_imu,
            10
        )
        self.imu_subscription
        self.odom_subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.callback_odom,
            10
        )
        self.odom_subscription

        self.publisher = self.create_publisher(
            PosYaw,
            "robot_data",
            10
        )

        self.yaw = 0
        self.prev_time = 0
        self.odom_msg_recv = False
        self.odom_msg = None
        self.pos_x = 0
        self.pos_y = 0


    def callback_imu(self, msg):
        # rot_z = math.atan2(2 * (msg.orientation.w * msg.orientation.z +  msg.orientation.x * msg.orientation.y),
        #                     1 - 2 * (msg.orientation.y ** 2 + msg.orientation.z ** 2 ))
        time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev_time == 0:
            self.prev_time = time

        self.yaw += msg.angular_velocity.z * (time - self.prev_time)
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.prev_time = time

        if self.odom_msg_recv == True:
            pos_x = self.odom_msg.pose.pose.position.x
            pos_y = self.odom_msg.pose.pose.position.y
            self.odom_msg_recv = False

            # self.get_logger().info('\nyaw = %f\n or %f\n pos_x = %f\n pos_y = %f\n' % (rot_z, self.yaw, pos_x, pos_y))

            pos_yaw = PosYaw()
            pos_yaw.x = pos_x
            pos_yaw.y = pos_y
            pos_yaw.yaw = self.yaw
            
            self.publisher.publish(pos_yaw)

    def callback_odom(self, msg):
        self.odom_msg_recv = True
        self.odom_msg = msg

def main():
    try:
        with rclpy.init():
            imu_node = ImuNode()

            rclpy.spin(imu_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()