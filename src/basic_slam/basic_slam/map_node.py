import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from messages.msg import PosYaw
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math

def bresenham_line(x0, y0, x1, y1):
    path = []

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)

    if x0 < x1:
        step_x = 1
    else:
        step_x = -1

    if y0 < y1:
        step_y = 1
    else:
        step_y = -1

    error = dx - dy

    while True:

        if x0 == x1 and y0 == y1:
            break
        
        path.append((x0, y0))
        
        e2 = 2 * error

        if e2 > -dy:
            error -= dy
            x0 += step_x
        
        if e2 < dx:
            error += dx
            y0 += step_y

    path = np.array(path)
    return path

class MapNode(Node):
    def __init__(self):
        super().__init__("map_node")
        self.pose_subscriber = self.create_subscription(
            PosYaw,
            "robot_data",
            self.pose_callback,
            10
        )
        self.pose_subscriber

        self.scan_subscriber = self.create_subscription(
            LaserScan, 
            "/scan",
            self.scan_callback,
            10
        )

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            "/map",
            qos
        )

        self.x = 0
        self.y = 0
        self.yaw = 0
        self.pose_msg_recv = False
        
        self.map_length = 10
        self.map_origin_x = self.map_length * -1 / 2
        self.map_origin_y = self.map_length * -1 / 2
        self.resolution = 0.05
        self.width = int(self.map_length / self.resolution)
        self.height = int(self.map_length / self.resolution)
        self.map_grid = np.full((self.height, self.width), 0, dtype=np.float32)

        
        self.occupancy_grid_msg = OccupancyGrid()
        self.occupancy_grid_msg.header.frame_id = 'odom'
        self.occupancy_grid_msg.info.map_load_time = self.get_clock().now().to_msg()
        self.occupancy_grid_msg.info.resolution = self.resolution
        self.occupancy_grid_msg.info.height = self.height
        self.occupancy_grid_msg.info.width = self.width
        self.occupancy_grid_msg.info.origin.position.x = float(self.map_origin_x)
        self.occupancy_grid_msg.info.origin.position.y = float(self.map_origin_y)
        self.occupancy_grid_msg.info.origin.position.z = 0.0
        self.occupancy_grid_msg.info.origin.orientation.w = 1.0
        self.occupancy_grid_msg.info.origin.orientation.x = 0.0
        self.occupancy_grid_msg.info.origin.orientation.y = 0.0
        self.occupancy_grid_msg.info.origin.orientation.z = 0.0
        

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.yaw = msg.yaw
        self.pose_msg_recv = True
        
    def scan_callback(self, msg):
        if self.pose_msg_recv == False:
            return
        
        ranges = msg.ranges
        coordinates = []
        
        for i, r in enumerate(ranges):
            if r > msg.range_min and r < msg.range_max:
                angle = msg.angle_min + msg.angle_increment * i
                rf_x = r * math.cos(angle)
                rf_y = r * math.sin(angle)

                x = self.x + rf_x * math.cos(self.yaw) - rf_y * math.sin(self.yaw)
                y = self.y + rf_x * math.sin(self.yaw) + rf_y * math.cos(self.yaw)
        
                coordinates.append((x, y))

        
        coordinates = np.array(coordinates)
        robot_grid_pose = [
            int((self.x - self.map_origin_x) // self.resolution),
            int((self.y - self.map_origin_y) // self.resolution)
        ]
        grid = np.floor((coordinates - [self.map_origin_x, self.map_origin_y]) / self.resolution).astype(int)
        
        grid[:, 1] = np.clip(grid[:, 1], 0, self.height - 1)
        grid[:, 0] = np.clip(grid[:, 0], 0, self.width - 1)

        for x, y in grid:
            path = bresenham_line(robot_grid_pose[0], robot_grid_pose[1], x, y)
            self.map_grid[path[:, 1], path[:, 0]] -= 0.1
        
        self.map_grid[grid[:, 1], grid[:, 0]] += 0.5
        self.map_grid = np.clip(self.map_grid, -3, 6,)

        probability = 1 - 1 / (1 + np.exp(self.map_grid))
        msg_grid = np.full((self.height, self.width), -1, dtype=np.int8)

        msg_grid[probability > 0.7] = 100
        msg_grid[probability < 0.2] = 0


        self.occupancy_grid_msg.data = msg_grid.flatten().tolist()
        self.occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()

        self.map_publisher.publish(self.occupancy_grid_msg)

        self.pose_msg_recv = False

    
    
def main():
    try:
        with rclpy.init():
            map_node = MapNode()

            rclpy.spin(map_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()