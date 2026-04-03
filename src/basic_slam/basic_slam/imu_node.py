import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Imu

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

        self.yaw = 0
        self.prev_time = 0
        self.vx = 0
        self.poz_x = 0
        self.calibrated = False
        self.bias_samples = []
        self.ax_bias = 0.0


    # def callback_imu(self, msg):
    #     rot_z = math.atan2(2 * (msg.orientation.w * msg.orientation.z +  msg.orientation.x * msg.orientation.y),
    #                         1 - 2 * (msg.orientation.y ** 2 + msg.orientation.z ** 2 ))
    #     time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    #     if self.prev_time == 0:
    #         self.prev_time = time

    #     if not self.calibrated:
    #         self.bias_samples.append(msg.linear_acceleration.x)

    #     if len(self.bias_samples) >= 100:
    #         self.ax_bias = sum(self.bias_samples) / len(self.bias_samples)
    #         self.calibrated = True
    #         self.get_logger().info(f"bias = {self.ax_bias:.5f}")
       

    #         self.yaw += msg.angular_velocity.z * (time - self.prev_time)
    #         self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
            
    #         self.vx += (msg.linear_acceleration.x - self.ax_bias) * (time - self.prev_time)
    #         self.poz_x += self.vx * (time - self.prev_time)

    #         self.prev_time = time
    #         self.get_logger().info('yaw = %f or %f\n poz_x = %f\n' % (rot_z, self.yaw, self.poz_x))

    def callback_imu(self, msg):
        rot_z = math.atan2(2 * (msg.orientation.w * msg.orientation.z +  msg.orientation.x * msg.orientation.y),
                            1 - 2 * (msg.orientation.y ** 2 + msg.orientation.z ** 2 ))
        time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev_time == 0:
            self.prev_time = time

        self.yaw += msg.angular_velocity.z * (time - self.prev_time)
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        
        # self.vx += (msg.linear_acceleration.x - self.ax_bias) * (time - self.prev_time)
        # self.poz_x += self.vx * (time - self.prev_time)

        self.prev_time = time
        # self.get_logger().info('yaw = %f or %f\n poz_x = %f\n' % (rot_z, self.yaw, self.poz_x))
        self.get_logger().info('yaw = %f or %f\n' % (rot_z, self.yaw))
    


def main():
    try:
        with rclpy.init():
            imu_node = ImuNode()

            rclpy.spin(imu_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()