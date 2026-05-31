import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.clock import Clock, ClockType

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

import numpy as np
import cv2
import time
import os


class MapMetrics(Node):
    """No-ground-truth 2D SLAM map quality metrics.

    Implements the three metrics from Filatov et al., "2D SLAM Quality
    Evaluation Methods" (arXiv:1708.02354), computed on the published /map
    OccupancyGrid. All three are *relative* indicators: lower is better, and
    they're meant to compare runs/algorithms on the same environment, not to
    give an absolute score.

      * proportion   - occupied cells / (free + unknown). Rises with blurred or
                       duplicated walls (trajectory mismatch).
      * corner_count - structural corners (Harris) after dropping noise dots.
                       Extra corners == doubled walls / artifacts.
      * enclosed     - free regions fully bordered by occupied/unknown cells.
                       Extra ones == superimposed rooms / loop-closure failures.

    Note: the published /map is hard-thresholded to {free, occupied, unknown},
    so the proportion metric here mainly catches *extra/duplicated* walls rather
    than grayscale blur (which needs the raw probability map).
    """

    def __init__(self):
        super().__init__("map_metrics")

        
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub = self.create_subscription(OccupancyGrid, "/map", self.callback, qos)

        self.pub_proportion = self.create_publisher(Float32, "/map_metrics/proportion", 10)
        self.pub_corners = self.create_publisher(Float32, "/map_metrics/corner_count", 10)
        self.pub_enclosed = self.create_publisher(Float32, "/map_metrics/enclosed_areas", 10)
       
        self.pub_image = self.create_publisher(Image, "/map_metrics/annotated", qos)

        self.annotated_path = os.path.expanduser("~/map_metrics.png")
        self.render_scale = 4

        self.min_component_area = 3    # drop sparse dots before corner detection
        self.min_enclosed_area = 10    # ignore tiny holes when counting enclosed areas

        # The grid is mostly unknown (e.g. 600x600 for a 30 m map). Crop to the
        # explored region (+ margin) before scoring so the unknown sea doesn't
        # dilute the proportion metric.
        self.roi_margin = 5          # cells of unknown to keep around the content

       
        self.idle_timeout = 2.0        
        self.latest_grid = None
        self.last_rx = None           
        self.reported = False
        self.create_timer(0.5, self.check_idle,
                           clock=Clock(clock_type=ClockType.STEADY_TIME))

    def callback(self, msg):
        w = msg.info.width
        h = msg.info.height
        if w == 0 or h == 0:
            return
       
        self.latest_grid = np.array(msg.data, dtype=np.int16).reshape(h, w)
        self.last_rx = time.monotonic()
        self.reported = False

    def check_idle(self):
        if (self.latest_grid is not None and not self.reported
                and self.last_rx is not None
                and (time.monotonic() - self.last_rx) > self.idle_timeout):
            self.report(self.latest_grid)
            self.reported = True

    def report(self, grid):
        grid = self._crop_to_known(grid, self.roi_margin)
        proportion = self.proportion(grid)
        corners = self._corner_points(grid)          # list of (x, y)
        enclosed = self._enclosed_contours(grid)     # list of contours

        self.pub_proportion.publish(Float32(data=float(proportion)))
        self.pub_corners.publish(Float32(data=float(len(corners))))
        self.pub_enclosed.publish(Float32(data=float(len(enclosed))))

        self.get_logger().info(
            f'final map metrics | proportion {proportion:.4f} | '
            f'corners {len(corners)} | enclosed {len(enclosed)}')

        annotated = self._render(grid, corners, enclosed)
        try:
            cv2.imwrite(self.annotated_path, annotated)
            self.get_logger().info(f'annotated map saved to {self.annotated_path}')
        except Exception as exc:                      
            self.get_logger().warn(f'could not save annotated map: {exc}')
        self.pub_image.publish(self._to_image_msg(annotated))

    # Metric A: proportion of occupied cells
    def proportion(self, grid):
        # Map to an occupancy-probability image; unknown (-1) is treated as free
        # (white), per the paper. Threshold = mean of all cells.
        prob = np.where(grid < 0, 0.0, grid.astype(np.float32) / 100.0)
        threshold = float(prob.mean())
        occupied = int(np.count_nonzero(prob > threshold))
        rest = prob.size - occupied  
        if rest == 0:
            return 0.0
        return occupied / rest

    # Metric B: structural corner count
    def corner_count(self, grid):
        return len(self._corner_points(grid))

    def _corner_points(self, grid):
        wall = (grid >= 50).astype(np.uint8)        # occupied cells
        wall = self._remove_small(wall, self.min_component_area)  # drop dots
        if wall.sum() == 0:
            return []

        img = (wall * 255).astype(np.float32)
        dst = cv2.cornerHarris(img, blockSize=3, ksize=3, k=0.04)
        if dst.max() <= 0.0:
            return []
        dst = cv2.dilate(dst, None)
        mask = (dst > 0.05 * dst.max()).astype(np.uint8)
        # Each cluster of corner response == one corner; use its centroid.
        num, _, _, centroids = cv2.connectedComponentsWithStats(mask)
        return [tuple(c) for c in centroids[1:]]    # skip background -> [(x, y), ...]

    # Metric C: enclosed areas
    def enclosed_areas(self, grid):
        return len(self._enclosed_contours(grid))

    def _enclosed_contours(self, grid):
        # An enclosed area is a free region fully bordered by occupied (or
        # unknown) cells -> a hole inside the "not free" foreground. We try both
        # unknown-as-wall and unknown-as-free and keep whichever yields more,
        # per the paper's iteration over undefined-region values.
        best = []
        for unknown_is_wall in (True, False):
            if unknown_is_wall:
                free = (grid == 0)
            else:
                free = (grid <= 0)
            foreground = (1 - free.astype(np.uint8))
            holes = self._hole_contours(foreground)
            if len(holes) > len(best):
                best = holes
        return best

    def _hole_contours(self, foreground):
        contours, hierarchy = cv2.findContours(
            foreground, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        if hierarchy is None:
            return []
        # node = [next, prev, first_child, parent]; a hole has a parent.
        return [contours[idx] for idx, node in enumerate(hierarchy[0])
                if node[3] != -1 and cv2.contourArea(contours[idx]) >= self.min_enclosed_area]

    # ------------------------------------------------------------------ #
    # Visualization
    # ------------------------------------------------------------------ #
    def _render(self, grid, corners, enclosed):
        # Base image: occupied -> black, free -> white, unknown -> gray.
        img = np.full((grid.shape[0], grid.shape[1], 3), 128, np.uint8)
        img[grid == 0] = (255, 255, 255)
        img[grid >= 50] = (0, 0, 0)

        # Enclosed areas: one translucent colour per region (+ outline). Draw
        # largest first so small regions (e.g. pillar cores) stay visible on top.
        if enclosed:
            regions = sorted(enclosed, key=cv2.contourArea, reverse=True)
            colors = self._distinct_colors(len(regions))
            overlay = img.copy()
            for contour, color in zip(regions, colors):
                cv2.drawContours(overlay, [contour], -1, color, cv2.FILLED)
            img = cv2.addWeighted(overlay, 0.45, img, 0.55, 0)
            for contour, color in zip(regions, colors):
                cv2.drawContours(img, [contour], -1, color, 1)

        # Corners: red dots.
        for x, y in corners:
            cv2.circle(img, (int(round(x)), int(round(y))), 1, (0, 0, 255), -1)

        s = self.render_scale
        img = cv2.resize(img, (img.shape[1] * s, img.shape[0] * s),
                         interpolation=cv2.INTER_NEAREST)
        # OccupancyGrid row 0 is at the origin (bottom); flip so it matches RViz.
        return cv2.flip(img, 0)

    @staticmethod
    def _distinct_colors(n):
        # Evenly spaced hues -> visually distinct BGR colours.
        n = max(n, 1)
        colors = []
        for i in range(n):
            hsv = np.uint8([[[int(179 * i / n), 220, 255]]])
            b, g, r = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
            colors.append((int(b), int(g), int(r)))
        return colors

    def _to_image_msg(self, bgr):
        msg = Image()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = bgr.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(msg.width * 3)
        msg.data = bgr.tobytes()
        return msg

    # ------------------------------------------------------------------ #
    @staticmethod
    def _crop_to_known(grid, margin):
        # Bounding box of all known (free or occupied) cells, padded by margin.
        known = np.argwhere(grid >= 0)
        if known.size == 0:
            return grid
        (y0, x0), (y1, x1) = known.min(axis=0), known.max(axis=0)
        y0 = max(int(y0) - margin, 0)
        x0 = max(int(x0) - margin, 0)
        y1 = min(int(y1) + margin + 1, grid.shape[0])
        x1 = min(int(x1) + margin + 1, grid.shape[1])
        return grid[y0:y1, x0:x1]

    @staticmethod
    def _remove_small(binary, min_area):
        num, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
        out = np.zeros_like(binary)
        for i in range(1, num):
            if stats[i, cv2.CC_STAT_AREA] >= min_area:
                out[labels == i] = 1
        return out


def main(args=None):
    node = None
    try:
        rclpy.init(args=args)
        node = MapMetrics()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Report on the last buffered map if the idle timer hadn't fired yet.
        if node is not None and node.latest_grid is not None and not node.reported:
            node.report(node.latest_grid)


if __name__ == '__main__':
    main()