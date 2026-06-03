# Autonomous Mobile Robotics Exam - Group 30

# Task 2 - Costmap sampling 

# Workflow:
    # Sample a random (x, y) in the map frame using the costmap.
        # Only at pixels where the cost is 0. 
    # Rejects any spot withiin VISITED_RADIUS of any previously attempted goal. 
    # favor nearby spots. It prefers ones that are closer to robot's current position.
        # Exponential weight formula 
    # Refuses pick spots too close to absolute edge of the map (within EDGE_MARGIN) to avoid Nav2 reaching out-of-bounds.

import math

import numpy as np

import rclpy.time
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid

from tiago_project_group30.constants import (
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

        # A memory list that stores the (x, y) coordinates of every location 
        # the robot has already tried to reach during the search phase.
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

        # convert the flat list of map pixels into a 2d matrix. 
        arr = np.array(grid.data, dtype=np.int8).reshape(h, w)

        # Free mask: cost == 0 (clearly navigable, not inflated, not
        # unknown).
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

        # Convert the free grid cells (rows/columns) into real-world map coordinates (meters)
        cxs = free[:, 1].astype(np.float64)
        cys = free[:, 0].astype(np.float64)
        xs = ox + (cxs + 0.5) * res
        ys = oy + (cys + 0.5) * res

        # Distance from robot (Euclidean).
        dists = np.hypot(xs - rx, ys - ry)

        # Remove any cells that are too close to places we've already been.
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

        #Filter the list to only include unvisited cells within our preferred maximum range
            # cells within SOFT_MAX_RANGE and not visited.
        candidate = not_visited & (dists <= SOFT_MAX_RANGE)
        if not np.any(candidate):
            # If there are no safe spots nearby, drop the range limit and look anywhere on the map
            candidate = not_visited
            if not np.any(candidate):
                return None  # Every spot visited. Reset the memory.

        cand_xs = xs[candidate]
        cand_ys = ys[candidate]
        cand_d = dists[candidate]

        # Weight = exp(-d / PREFERRED_RANGE). 
        # Assign a probability weight to each valid spot based on distance.
        # Closer spots get a higher weight, making them more likely to be picked.
        weights = np.exp(-cand_d / PREFERRED_RANGE)
        wsum = weights.sum()
        if wsum <= 0.0 or not np.isfinite(wsum):
            return None
        
        # Convert weights into probabilities
        probs = weights / wsum

        # Randomly select one index from the list, biased by calculated probabilities
        idx = int(np.random.choice(len(cand_xs), p=probs))

        # Extract the final chosen coordinates
        x_goal = float(cand_xs[idx])
        y_goal = float(cand_ys[idx])
        yaw = math.atan2(y_goal - ry, x_goal - rx)

        return (x_goal, y_goal,yaw)
