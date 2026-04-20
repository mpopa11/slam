import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import TransformBroadcaster

from messages.msg import PosYaw
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped

import numpy as np
import math
import small_gicp

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

    return np.array(path)

def htm(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    transform = np.array([
        [c, -s, 0, x],
        [s,  c, 0, y],
        [0,  0, 1, 0],
        [0,  0, 0, 1]
    ])

    return transform

def get_normalized_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

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

        self.transform_broadcaster = TransformBroadcaster(self)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            "/map",
            qos
        )

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.last_odom_x = None
        self.last_odom_y = None
        self.last_odom_yaw = None

        self.pose_msg_recv = False

        self.x_icp = 0.0
        self.y_icp = 0.0
        self.yaw_icp = None
        
        self.map_length = 10
        self.map_origin_x = self.map_length * -1 / 2
        self.map_origin_y = self.map_length * -1 / 2
        self.resolution = 0.05
        self.width = int(self.map_length / self.resolution)
        self.height = int(self.map_length / self.resolution)
        self.map_grid = np.full((self.height, self.width), 0, dtype=np.float32)

        
        self.occupancy_grid_msg = OccupancyGrid()
        self.occupancy_grid_msg.header.frame_id = 'map'
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


        self.map_odom_tf_msg = TransformStamped()
        self.map_odom_tf_msg.header.frame_id = "/map"
        self.map_odom_tf_msg.child_frame_id = "/odom"

        self.tf_timer = self.create_timer(0.1, self.publish_map_odom_tf)

    def pose_callback(self, msg):
        self.odom_x = msg.x
        self.odom_y = msg.y
        self.odom_yaw = msg.yaw
        self.pose_msg_recv = True
        
    def scan_callback(self, msg):
        if self.pose_msg_recv == False:
            return
        
        ranges = msg.ranges
        local_coordinates = []
        
        for i, r in enumerate(ranges):
            if r > msg.range_min and r < msg.range_max:
                angle = msg.angle_min + msg.angle_increment * i
                rf_x = r * math.cos(angle)
                rf_y = r * math.sin(angle)

                local_coordinates.append((rf_x, rf_y))

        local_coordinates = np.array(local_coordinates)

        if self.yaw_icp is None:
            self.x_icp = self.odom_x
            self.y_icp = self.odom_y
            self.yaw_icp = self.odom_yaw
            self.last_odom_x = self.odom_x
            self.last_odom_y = self.odom_y
            self.last_odom_yaw = self.odom_yaw

        pred_x, pred_y, pred_yaw = self.predict_pose()
        
        map_point_cloud = self.extract_map()

        if map_point_cloud is None:
            self.x_icp = pred_x
            self.y_icp = pred_y
            self.yaw_icp = pred_yaw
        else:
            
            x = pred_x + local_coordinates[:, 0] * math.cos(pred_yaw) - local_coordinates[:, 1] * math.sin(pred_yaw)
            y = pred_y + local_coordinates[:, 0] * math.sin(pred_yaw) + local_coordinates[:, 1] * math.cos(pred_yaw)

            world_coordinates = np.column_stack([x, y, np.zeros(len(x))])

            (target, target_kd_tree) = small_gicp.preprocess_points(map_point_cloud, downsampling_resolution=0.025)
            (source, _) = small_gicp.preprocess_points(world_coordinates, downsampling_resolution=0.025)
            result = small_gicp.align(target, source, target_kd_tree,
                                    registration_type='GICP',
                                    max_correspondence_distance=0.3, 
                                    max_iterations=30)


            if (result.converged 
                and math.hypot(result.T_target_source[0, 3], result.T_target_source[1, 3]) < 0.3 
                and abs(math.atan2(result.T_target_source[1, 0], result.T_target_source[0, 0])) < 0.26):
                
                true_transform = result.T_target_source @ htm(pred_x, pred_y, pred_yaw)
                self.x_icp, self.y_icp = true_transform[:2, 3]
                self.yaw_icp = math.atan2(math.sin(math.atan2(true_transform[1, 0], true_transform[0, 0])),
                                        math.cos(math.atan2(true_transform[1, 0], true_transform[0, 0])))
                # self.get_logger().info('converged')
                
            else:
                self.x_icp = pred_x
                self.y_icp = pred_y
                self.yaw_icp = pred_yaw
            # self.get_logger().info(str(list(zip(x,y))))
            # self.get_logger().info(f'{self.odom_x}\t{self.odom_y}\t{self.odom_yaw}')
            # self.get_logger().info(f'{self.x_icp}\t{self.y_icp}\t{self.yaw_icp}')
            self.get_logger().info(
            f'odom {self.odom_x:+.3f} {self.odom_y:+.3f} {self.odom_yaw:+.3f} | '
            f'icp  {self.x_icp:+.3f} {self.y_icp:+.3f} {self.yaw_icp:+.3f}')

        self.last_odom_x = self.odom_x
        self.last_odom_y = self.odom_y
        self.last_odom_yaw = self.odom_yaw

        robot_grid_pose = [
            int((self.x_icp - self.map_origin_x) // self.resolution),
            int((self.y_icp - self.map_origin_y) // self.resolution)
        ]

        x = self.x_icp + local_coordinates[:, 0] * math.cos(self.yaw_icp) - local_coordinates[:, 1] * math.sin(self.yaw_icp)
        y = self.y_icp + local_coordinates[:, 0] * math.sin(self.yaw_icp) + local_coordinates[:, 1] * math.cos(self.yaw_icp)

        world_coordinates = np.column_stack([x, y, np.zeros(len(x))])

        grid = np.floor((world_coordinates[:, :2] - [self.map_origin_x, self.map_origin_y]) / self.resolution).astype(int)

        grid[:, 1] = np.clip(grid[:, 1], 0, self.height - 1)
        grid[:, 0] = np.clip(grid[:, 0], 0, self.width - 1)

        for x, y in grid:
            path = bresenham_line(robot_grid_pose[0], robot_grid_pose[1], x, y)
            if len(path) > 0:
                self.map_grid[path[:, 1], path[:, 0]] -= 0.05
        
        self.map_grid[grid[:, 1], grid[:, 0]] += 0.2
        self.map_grid = np.clip(self.map_grid, -3, 6,)

        probability = 1 - 1 / (1 + np.exp(self.map_grid))
        msg_grid = np.full((self.height, self.width), -1, dtype=np.int8)

        msg_grid[probability > 0.7] = 100
        msg_grid[probability < 0.2] = 0

        self.occupancy_grid_msg.data = msg_grid.flatten().tolist()
        self.occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_odom_tf_msg.header.stamp = self.occupancy_grid_msg.header.stamp

        self.map_publisher.publish(self.occupancy_grid_msg)
        self.publish_map_odom_tf()

        self.pose_msg_recv = False

    
    def predict_pose(self):
        d_x1 = self.odom_x - self.last_odom_x
        d_y1 = self.odom_y - self.last_odom_y
        d_yaw = get_normalized_angle(self.odom_yaw - self.last_odom_yaw)

        c, s = math.cos(self.last_odom_yaw), math.sin(self.last_odom_yaw)
        d_x = c * d_x1 + s * d_y1
        d_y = c * d_y1 - s * d_x1
        
        c, s = math.cos(self.yaw_icp), math.sin(self.yaw_icp)
        x = self.x_icp + c * d_x - s * d_y 
        y = self.y_icp + c * d_y + s * d_x
        yaw = get_normalized_angle(self.yaw_icp + d_yaw)
        return x, y, yaw

    def extract_map(self):
        points = np.argwhere(self.map_grid > 3.0)

        if len(points) == 0:
            return None

        x = self.resolution * (points[:, 1] + 0.5) + self.map_origin_x
        y = self.resolution * (points[:, 0] + 0.5) + self.map_origin_y

        return np.column_stack([x, y, np.zeros(len(x))])
    
    def publish_map_odom_tf(self):

        if self.yaw_icp is None:
            return

        map_odom_tf = htm(self.x_icp, self.y_icp, self.yaw_icp) @ np.linalg.inv(htm(self.odom_x, self.odom_y, self.odom_yaw))

        x = map_odom_tf[1, 3]
        y = map_odom_tf[2, 3]
        yaw =  math.atan2(math.sin(math.atan2(map_odom_tf[1, 0], map_odom_tf[0, 0])),
                                        math.cos(math.atan2(map_odom_tf[1, 0], map_odom_tf[0, 0])))
        
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)

        # self.map_odom_tf_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_odom_tf_msg.transform.translation.x = float(x)
        self.map_odom_tf_msg.transform.translation.y = float(y)
        self.map_odom_tf_msg.transform.translation.z = 0.0

        self.map_odom_tf_msg._transform.rotation.x = 0.0
        self.map_odom_tf_msg._transform.rotation.y = 0.0
        self.map_odom_tf_msg._transform.rotation.z = float(qz)
        self.map_odom_tf_msg._transform.rotation.w = float(qw)

        self.transform_broadcaster.sendTransform(self.map_odom_tf_msg)
    
def main():
    try:
        with rclpy.init():
            map_node = MapNode()
            
            rclpy.spin(map_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
