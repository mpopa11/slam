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
import cv2
from collections import deque

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
        
        self.map_length = 30
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
        self.map_odom_tf_msg.header.frame_id = "map"
        self.map_odom_tf_msg.child_frame_id = "odom"

        # self.tf_timer = self.create_timer(0.1, self.publish_map_odom_tf)

        # Lidar mounting offset in base_link frame (from URDF / launch static TF).
        # base_scan is at (-0.032, 0, 0.172) relative to base_link
        self.laser_offset_x = -0.032
        self.laser_offset_y = 0.0

        # Time-stamped pose buffer for per-beam deskew.
        self.pose_buffer = deque(maxlen=400)
        self.pose_buffer_max_age = 1.0  # seconds

        # Previous scan-end ICP pose and time, used to re-deskew with the
        # corrected trajectory on the second ICP pass.
        self.t_end_prev = None
        self.x_icp_prev = None
        self.y_icp_prev = None
        self.yaw_icp_prev = None

    def pose_callback(self, msg):
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pose_buffer.append((now, msg.x, msg.y, msg.yaw))
        # Drop entries older than max_age relative to the newest sample.
        while self.pose_buffer and (now - self.pose_buffer[0][0]) > self.pose_buffer_max_age:
            self.pose_buffer.popleft()
        self.odom_x = msg.x
        self.odom_y = msg.y
        self.odom_yaw = msg.yaw
        self.pose_msg_recv = True

    def interpolate_pose(self, t):
        """Linearly interpolate (x, y, yaw) from pose_buffer at time t (seconds).
        Clamps to the buffer endpoints when t is outside. Returns None if empty."""
        buf = self.pose_buffer
        n = len(buf)
        if n == 0:
            return None
        if n == 1 or t <= buf[0][0]:
            return buf[0][1], buf[0][2], buf[0][3]
        if t >= buf[-1][0]:
            return buf[-1][1], buf[-1][2], buf[-1][3]
        # Linear scan from the right (typical query times are near the latest sample).
        for i in range(n - 1, 0, -1):
            t1, x1, y1, yaw1 = buf[i]
            t0, x0, y0, yaw0 = buf[i - 1]
            if t0 <= t <= t1:
                a = (t - t0) / (t1 - t0)
                x = x0 + a * (x1 - x0)
                y = y0 + a * (y1 - y0)
                dyaw = get_normalized_angle(yaw1 - yaw0)
                yaw = get_normalized_angle(yaw0 + a * dyaw)
                return x, y, yaw
        return buf[-1][1], buf[-1][2], buf[-1][3]
        
    def scan_callback(self, msg):
        if self.pose_msg_recv == False:
            return
        
        # ranges = msg.ranges
        # local_coordinates = []
        
        # for i, r in enumerate(ranges):
        #     if r > msg.range_min and r < msg.range_max:
        #         angle = msg.angle_min + msg.angle_increment * i
        #         rf_x = r * math.cos(angle)
        #         rf_y = r * math.sin(angle)

        #         local_coordinates.append((rf_x, rf_y))

        # local_coordinates = np.array(local_coordinates)

        n = len(msg.ranges)
        ranges = np.asarray(msg.ranges, dtype=np.float32)


        scan_start_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if msg.time_increment > 0.0:
            ti = msg.time_increment
        elif msg.scan_time > 0.0:
            ti = msg.scan_time / max(n, 1)
        else:
            ti = 0.1 / max(n, 1)  # assume 10 Hz fallback
        t_beams = scan_start_t + np.arange(n) * ti
        t_end = scan_start_t + (n - 1) * ti

        pose_end = self.interpolate_pose(t_end)
        if pose_end is None:
            self.pose_msg_recv = False
            return
        self.odom_x, self.odom_y, self.odom_yaw = pose_end

        # Per-beam world pose by interpolation
        buf = self.pose_buffer
        bt = np.array([e[0] for e in buf], dtype=np.float64)
        bx_w = np.array([e[1] for e in buf], dtype=np.float64)
        by_w = np.array([e[2] for e in buf], dtype=np.float64)
        byaw = np.array([e[3] for e in buf], dtype=np.float64)
        # np.searchsorted gives the right-side index; clamp into [1, len-1].
        idx = np.clip(np.searchsorted(bt, t_beams, side='right'), 1, len(bt) - 1)
        t0 = bt[idx - 1]
        t1 = bt[idx]
        dt = np.where(t1 > t0, t1 - t0, 1.0)
        a = np.clip((t_beams - t0) / dt, 0.0, 1.0)
        x_i = bx_w[idx - 1] + a * (bx_w[idx] - bx_w[idx - 1])
        y_i = by_w[idx - 1] + a * (by_w[idx] - by_w[idx - 1])
        dyaw_seg = np.arctan2(np.sin(byaw[idx] - byaw[idx - 1]),
                              np.cos(byaw[idx] - byaw[idx - 1]))
        yaw_i = byaw[idx - 1] + a * dyaw_seg

        # Beam points in beam-time body (base_link) frame.
        # Endpoint in laser (base_scan) frame, then translated by the static
        # base_link <- base_scan offset. base_scan has no rotation relative to
        # base_link (per launch file), so only a translation is needed.
        angles = msg.angle_min + msg.angle_increment * np.arange(n)
        valid = (ranges > msg.range_min) & (ranges < msg.range_max) & np.isfinite(ranges)
        bx = ranges * np.cos(angles) + self.laser_offset_x
        by = ranges * np.sin(angles) + self.laser_offset_y

        # Beam-time body -> world.
        ci, si = np.cos(yaw_i), np.sin(yaw_i)
        wx = ci * bx - si * by + x_i
        wy = si * bx + ci * by + y_i

        # World -> scan-end body frame (so the rest of the pipeline treats the
        # cloud as if all beams were captured at t_end).
        ce, se = math.cos(self.odom_yaw), math.sin(self.odom_yaw)
        dx_w = wx - self.odom_x
        dy_w = wy - self.odom_y
        px =  ce * dx_w + se * dy_w
        py = -se * dx_w + ce * dy_w

        local_coordinates = np.column_stack([px[valid], py[valid]])

        if self.yaw_icp is None:
            self.x_icp = self.odom_x
            self.y_icp = self.odom_y
            self.yaw_icp = self.odom_yaw
            self.last_odom_x = self.odom_x
            self.last_odom_y = self.odom_y
            self.last_odom_yaw = self.odom_yaw

            # self.pose_msg_recv = False
            # return

        # d_dist = math.hypot(self.odom_x - self.last_odom_x, self.odom_y - self.last_odom_y)
        # d_angle = abs(get_normalized_angle(self.odom_yaw - self.last_odom_yaw))
        # if d_dist < 0.05 and d_angle < 0.05:
        #     self.pose_msg_recv = False
        #     return

        pred_x, pred_y, pred_yaw = self.predict_pose()
        
        if len(local_coordinates) == 0:
            self.x_icp = pred_x
            self.y_icp = pred_y
            self.yaw_icp = pred_yaw

            self.last_odom_x = self.odom_x
            self.last_odom_y = self.odom_y
            self.last_odom_yaw = self.odom_yaw

            self.pose_msg_recv = False
            return

        map_point_cloud = self.extract_map()

        if map_point_cloud is None:
            self.x_icp = pred_x
            self.y_icp = pred_y
            self.yaw_icp = pred_yaw
        else:
            # ---- First pass: ICP with odom-based deskew ----
            x = pred_x + local_coordinates[:, 0] * math.cos(pred_yaw) - local_coordinates[:, 1] * math.sin(pred_yaw)
            y = pred_y + local_coordinates[:, 0] * math.sin(pred_yaw) + local_coordinates[:, 1] * math.cos(pred_yaw)
            world_coordinates = np.column_stack([x, y, np.zeros(len(x))])

            (target, target_kd_tree) = small_gicp.preprocess_points(map_point_cloud, downsampling_resolution=0.025)
            (source, _) = small_gicp.preprocess_points(world_coordinates, downsampling_resolution=0.025)
            result = small_gicp.align(target, source, target_kd_tree,
                                    registration_type='GICP',
                                    max_correspondence_distance=0.2,
                                    max_iterations=15)

            icp_accepted = (result.converged
                and math.hypot(result.T_target_source[0, 3], result.T_target_source[1, 3]) < 0.2
                and abs(math.atan2(result.T_target_source[1, 0], result.T_target_source[0, 0])) < 0.09)

            if icp_accepted:
                true_transform = result.T_target_source @ htm(pred_x, pred_y, pred_yaw)
                self.x_icp, self.y_icp = true_transform[:2, 3]
                self.yaw_icp = get_normalized_angle(
                    math.atan2(true_transform[1, 0], true_transform[0, 0]))
            else:
                self.x_icp = pred_x
                self.y_icp = pred_y
                self.yaw_icp = pred_yaw

            # ---- Second pass: re-deskew using ICP-corrected trajectory between
            # the previous and current scan-end poses, then re-run ICP. ----
            if (icp_accepted
                and self.t_end_prev is not None
                and self.x_icp_prev is not None
                and (t_end - self.t_end_prev) > 1e-3):

                dt_scan = t_end - self.t_end_prev
                a_beam = np.clip((t_beams - self.t_end_prev) / dt_scan, 0.0, 1.0)
                dyaw_total = get_normalized_angle(self.yaw_icp - self.yaw_icp_prev)

                x_b2 = self.x_icp_prev + a_beam * (self.x_icp - self.x_icp_prev)
                y_b2 = self.y_icp_prev + a_beam * (self.y_icp - self.y_icp_prev)
                yaw_b2 = self.yaw_icp_prev + a_beam * dyaw_total

                # Beam-time body -> world via ICP trajectory.
                cb2, sb2 = np.cos(yaw_b2), np.sin(yaw_b2)
                wx2 = cb2 * bx - sb2 * by + x_b2
                wy2 = sb2 * bx + cb2 * by + y_b2

                # World -> scan-end body using the just-corrected ICP pose.
                ce2, se2 = math.cos(self.yaw_icp), math.sin(self.yaw_icp)
                dx_w2 = wx2 - self.x_icp
                dy_w2 = wy2 - self.y_icp
                px2 =  ce2 * dx_w2 + se2 * dy_w2
                py2 = -se2 * dx_w2 + ce2 * dy_w2
                local_coordinates_2 = np.column_stack([px2[valid], py2[valid]])

                if len(local_coordinates_2) > 0:
                    cw, sw = math.cos(self.yaw_icp), math.sin(self.yaw_icp)
                    x2 = self.x_icp + local_coordinates_2[:, 0] * cw - local_coordinates_2[:, 1] * sw
                    y2 = self.y_icp + local_coordinates_2[:, 0] * sw + local_coordinates_2[:, 1] * cw
                    world_coordinates_2 = np.column_stack([x2, y2, np.zeros(len(x2))])

                    (source2, _) = small_gicp.preprocess_points(
                        world_coordinates_2, downsampling_resolution=0.025)
                    # Tighter correspondence distance + fewer iters: this is a refinement.
                    result2 = small_gicp.align(target, source2, target_kd_tree,
                                            registration_type='GICP',
                                            max_correspondence_distance=0.1,
                                            max_iterations=10)

                    if (result2.converged
                        and math.hypot(result2.T_target_source[0, 3], result2.T_target_source[1, 3]) < 0.1
                        and abs(math.atan2(result2.T_target_source[1, 0], result2.T_target_source[0, 0])) < 0.05):

                        true_transform2 = result2.T_target_source @ htm(self.x_icp, self.y_icp, self.yaw_icp)
                        self.x_icp, self.y_icp = true_transform2[:2, 3]
                        self.yaw_icp = get_normalized_angle(
                            math.atan2(true_transform2[1, 0], true_transform2[0, 0]))
                        # Use the refined cloud for the map update below.
                        local_coordinates = local_coordinates_2

            self.get_logger().info(
            f'odom {self.odom_x:+.3f} {self.odom_y:+.3f} {self.odom_yaw:+.3f} | '
            f'icp  {self.x_icp:+.3f} {self.y_icp:+.3f} {self.yaw_icp:+.3f} | '
            f'pred {pred_x:+.3f} {pred_y:+.3f} {pred_yaw:+.3f}')

        self.last_odom_x = self.odom_x
        self.last_odom_y = self.odom_y
        self.last_odom_yaw = self.odom_yaw

        robot_grid_pose = [
            int((self.x_icp - self.map_origin_x) // self.resolution),
            int((self.y_icp - self.map_origin_y) // self.resolution)
        ]

        robot_grid_pose[0] = np.clip(robot_grid_pose[0], 0, self.width - 1)
        robot_grid_pose[1] = np.clip(robot_grid_pose[1], 0, self.height - 1)

        x = self.x_icp + local_coordinates[:, 0] * math.cos(self.yaw_icp) - local_coordinates[:, 1] * math.sin(self.yaw_icp)
        y = self.y_icp + local_coordinates[:, 0] * math.sin(self.yaw_icp) + local_coordinates[:, 1] * math.cos(self.yaw_icp)

        world_coordinates = np.column_stack([x, y, np.zeros(len(x))])

        grid = np.floor((world_coordinates[:, :2] - [self.map_origin_x, self.map_origin_y]) / self.resolution).astype(int)

        grid[:, 1] = np.clip(grid[:, 1], 0, self.height - 1)
        grid[:, 0] = np.clip(grid[:, 0], 0, self.width - 1)
        
        free_count = np.zeros((self.height, self.width), dtype=np.int32)
        rx, ry = robot_grid_pose[0], robot_grid_pose[1]
        for gx, gy in grid:
            pts = bresenham_line(rx, ry, int(gx), int(gy))
            if len(pts) == 0:
                continue
            xs, ys = pts[:, 0], pts[:, 1]
            m = (xs >= 0) & (xs < self.width) & (ys >= 0) & (ys < self.height)
            np.add.at(free_count, (ys[m], xs[m]), 1)

        self.map_grid -= free_count.astype(np.float32) * 0.1
        self.map_grid[grid[:, 1], grid[:, 0]] += 0.2
        self.map_grid = np.clip(self.map_grid, -6, 6)

        probability = 1 - 1 / (1 + np.exp(self.map_grid))
        msg_grid = np.full((self.height, self.width), -1, dtype=np.int8)

        msg_grid[probability > 0.7] = 100
        msg_grid[probability < 0.2] = 0

        self.occupancy_grid_msg.data = msg_grid.flatten().tolist()
        self.occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_odom_tf_msg.header.stamp = self.occupancy_grid_msg.header.stamp

        self.map_publisher.publish(self.occupancy_grid_msg)
        self.publish_map_odom_tf()

        # Stash scan-end ICP pose for the next scan's second-pass deskew.
        self.t_end_prev = t_end
        self.x_icp_prev = self.x_icp
        self.y_icp_prev = self.y_icp
        self.yaw_icp_prev = self.yaw_icp

        self.pose_msg_recv = False

    
    def predict_pose(self):
        d_x1 = self.odom_x - self.last_odom_x
        d_y1 = self.odom_y - self.last_odom_y
        d_yaw = get_normalized_angle(self.odom_yaw - self.last_odom_yaw)

        # if abs(d_x1) < 1e-6 and abs(d_y1) < 1e-6 and abs(d_yaw) < 1e-6:
        #     return self.x_icp, self.y_icp, self.yaw_icp

        c, s = math.cos(self.last_odom_yaw), math.sin(self.last_odom_yaw)
        d_x = c * d_x1 + s * d_y1
        d_y = c * d_y1 - s * d_x1
        
        c, s = math.cos(self.yaw_icp), math.sin(self.yaw_icp)
        x = self.x_icp + c * d_x - s * d_y 
        y = self.y_icp + c * d_y + s * d_x
        yaw = get_normalized_angle(self.yaw_icp + d_yaw)
        return x, y, yaw

    def extract_map(self):
        points = np.argwhere(self.map_grid > 1.0)

        if len(points) == 0:
            return None

        x = self.resolution * (points[:, 1] + 0.5) + self.map_origin_x
        y = self.resolution * (points[:, 0] + 0.5) + self.map_origin_y

        return np.column_stack([x, y, np.zeros(len(x))])
    
    def publish_map_odom_tf(self):

        if self.yaw_icp is None:
            return

        map_odom_tf = htm(self.x_icp, self.y_icp, self.yaw_icp) @ np.linalg.inv(htm(self.odom_x, self.odom_y, self.odom_yaw))

        x, y = map_odom_tf[:2, 3]
        yaw =  math.atan2(math.sin(math.atan2(map_odom_tf[1, 0], map_odom_tf[0, 0])),
                                        math.cos(math.atan2(map_odom_tf[1, 0], map_odom_tf[0, 0])))
        
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)

        self.map_odom_tf_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_odom_tf_msg.transform.translation.x = float(x)
        self.map_odom_tf_msg.transform.translation.y = float(y)
        self.map_odom_tf_msg.transform.translation.z = 0.0

        self.map_odom_tf_msg.transform.rotation.x = 0.0
        self.map_odom_tf_msg.transform.rotation.y = 0.0
        self.map_odom_tf_msg.transform.rotation.z = float(qz)
        self.map_odom_tf_msg.transform.rotation.w = float(qw)

        self.transform_broadcaster.sendTransform(self.map_odom_tf_msg)
    
def main(args=None):
    try:
        rclpy.init(args=args)
        map_node = MapNode()
        
        rclpy.spin(map_node)
        map_node.destroy_node()

        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
