# TASK 2 of Autonomous Mobile Robotics Exam - Group 30
#
# CostmapSampler: subscribes to /global_costmap/costmap and serves random
# (x, y) waypoints for the State 3 random search.
#
# Sampling strategy:
#   - Accept ONLY cells with cost == 0 in the global costmap (clearly
#     free, not inflated, not unknown). The global planner decides if a
#     candidate is actually reachable; if it can't plan a path, Nav2
#     reports failure and the state machine asks for the next sample.
#   - Reject cells within VISITED_RADIUS of any previously-attempted
#     waypoint (so the search spreads instead of looping back).
#   - Weight remaining candidates by exp(-d/PREFERRED_RANGE) so close
#     cells dominate; hard-cap at SOFT_MAX_RANGE unless nothing closer
#     is unvisited.
#   - Drop cells within EDGE_MARGIN of the map boundary so the global
#     NavFn planner doesn't reach out-of-bounds pixels (which would spam
#     "worldToMap failed" tens of thousands of times per plan attempt).

import math

import numpy as np

import rclpy.time
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid

from tiago_project_group30.task2_constants import (
    EDGE_MARGIN,
    PREFERRED_RANGE,
    SOFT_MAX_RANGE,
    VISITED_RADIUS,
)


class CostmapSampler:
    def __init__(self, node, tf_buffer, cb_group_io,
                 map_frame: str = "map", robot_base_frame: str = "base_link"):
        self.node = node
        self.tf_buffer = tf_buffer
        self.map_frame = map_frame
        self.robot_base_frame = robot_base_frame

        # The costmap is published with TRANSIENT_LOCAL durability so we
        # use the same QoS to make sure we get the latched message.
        self.costmap_msg = None
        costmap_qos = QoSProfile(depth=1)
        costmap_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        costmap_qos.reliability = ReliabilityPolicy.RELIABLE
        node.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self._costmap_cb,
            costmap_qos,
            callback_group=cb_group_io,
        )

        # visited_waypoints stores (x, y) of every waypoint we have
        # already *attempted* (whether reached or timed out). Acts as
        # memory so the sampler doesn't repeatedly pick nearby cells.
        self.visited_waypoints = []

    def _costmap_cb(self, msg: OccupancyGrid):
        self.costmap_msg = msg

    # ------------------------------------------------------------------
    def sample_random_xy(self):
        # Returns (x, y) in MAP frame or None if no usable cell is found.
        if self.costmap_msg is None:
            return None
        grid = self.costmap_msg
        w = grid.info.width
        h = grid.info.height
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        arr = np.array(grid.data, dtype=np.int8).reshape(h, w)

        # Free mask: cost == 0 (clearly navigable, not inflated, not
        # unknown). The planner handles connectivity.
        free_mask = arr == 0

        # Drop cells too close to the map boundary so the global NavFn
        # planner doesn't reach out-of-bounds pixels.
        margin_cells = int(math.ceil(EDGE_MARGIN / res))
        if margin_cells > 0:
            free_mask[:margin_cells, :] = False
            free_mask[-margin_cells:, :] = False
            free_mask[:, :margin_cells] = False
            free_mask[:, -margin_cells:] = False

        free = np.argwhere(free_mask)
        if len(free) == 0:
            return None

        # Robot pose in map frame (for distance weighting)
        try:
            tf_robot = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception:
            return None
        rx = tf_robot.transform.translation.x
        ry = tf_robot.transform.translation.y

        # Vectorise: every free cell -> (x, y) in map frame.
        cxs = free[:, 1].astype(np.float64)
        cys = free[:, 0].astype(np.float64)
        xs = ox + (cxs + 0.5) * res
        ys = oy + (cys + 0.5) * res

        # Distance from robot (Euclidean).
        dists = np.hypot(xs - rx, ys - ry)

        # Visited rejection (vectorised via broadcasting).
        if self.visited_waypoints:
            vx = np.array([p[0] for p in self.visited_waypoints])
            vy = np.array([p[1] for p in self.visited_waypoints])
            min_visited = np.min(
                np.hypot(xs[:, None] - vx[None, :], ys[:, None] - vy[None, :]),
                axis=1,
            )
            not_visited = min_visited >= VISITED_RADIUS
        else:
            not_visited = np.ones(len(xs), dtype=bool)

        # First try: cells within SOFT_MAX_RANGE and not visited.
        candidate = not_visited & (dists <= SOFT_MAX_RANGE)
        if not np.any(candidate):
            # No nearby unvisited cells -> drop the hard cap and accept
            # any unvisited cell.
            candidate = not_visited
            if not np.any(candidate):
                return None  # area saturated -> caller resets memory

        cand_xs = xs[candidate]
        cand_ys = ys[candidate]
        cand_d = dists[candidate]

        # Weight = exp(-d / PREFERRED_RANGE). Cells at PREFERRED_RANGE
        # are ~37% as likely as cells at the robot's foot, etc.
        weights = np.exp(-cand_d / PREFERRED_RANGE)
        wsum = weights.sum()
        if wsum <= 0.0 or not np.isfinite(wsum):
            return None
        probs = weights / wsum

        idx = int(np.random.choice(len(cand_xs), p=probs))

        x_goal = float(cand_xs[idx])
        y_goal = float(cand_ys[idx])
        yaw = math.atan2(y_goal - ry, x_goal - rx)

        return (x_goal, y_goal,yaw)
