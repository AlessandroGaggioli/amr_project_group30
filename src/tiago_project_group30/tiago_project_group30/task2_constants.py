# TASK 2 of Autonomous Mobile Robotics Exam - Group 30
#
# Module-level constants shared by all task2_* components. Centralized so
# tuning a parameter touches exactly one file. Values and comments are
# preserved verbatim from the original monolithic task2_manager.py.

##############################
# Tiago parameters
##############################

JOINT_ARM_NAMES = [
    "torso_lift_joint",
    "arm_1_joint",
    "arm_2_joint",
    "arm_3_joint",
    "arm_4_joint",
    "arm_5_joint",
    "arm_6_joint",
    "arm_7_joint",
]

# Same HOME configuration used in task1_manager: torso lifted + arm tucked.
HOME_JOINT_POSITIONS = [0.15, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]

##############################
# Search strategy
##############################

# Memory radius: when sampling a new waypoint we reject any (x, y) that is
# within this distance of a previously-tried waypoint. The 2-yaw spin at
# each waypoint already scans a ~3 m FOV around the spot, so 1.5 m keeps
# adjacent waypoints non-overlapping and pushes the search to spread
# faster across the room.
VISITED_RADIUS = 1.5  # meters

# Sampling is BIASED toward cells close to the robot's current position.
# - PREFERRED_RANGE: cells beyond this get exponentially less likely.
# - SOFT_MAX_RANGE: hard cap; cells beyond this are excluded entirely
#   unless no closer cell is available (then the cap is dropped).
# This produces "expanding ring" exploration: the robot first surveys
# nearby cells, only jumping farther once close cells are visited.
PREFERRED_RANGE = 2.0  # meters
SOFT_MAX_RANGE = 3.0   # meters

# Don't sample cells closer than this many meters to the saved-map edge.
# The global NavFn planner expands a potential field around the goal that
# reaches a few inflation radii outward; if that expansion crosses the map
# boundary it logs "worldToMap failed: mx,my: ..., size_x,size_y: ..." once
# *per iteration* and we get tens of thousands of errors per plan attempt.
# Must be >= the global_costmap inflation_radius (now 0.7 m in tiago_nav2.yaml)
# so the planner never reaches out-of-bounds pixels.
EDGE_MARGIN = 0.8  # meters

##############################
# ArUco / approach parameters
##############################

PICK_MARKER_FRAME_RAW = "aruco_pick_frame"          # published by aruco_single ID 26
PLACE_MARKER_FRAME_RAW = "aruco_place_frame"        # published by aruco_single ID 238

# Derived approach TFs we broadcast (lab3-style naming).
PICK_APPROACH_FRAME = "aruco_pick_approach"
PLACE_APPROACH_FRAME = "aruco_place_approach"

# Camera optical frame (same as the camera_frame parameter of aruco_single).
CAMERA_FRAME = "head_front_camera_rgb_optical_frame"

# Distance the robot stops in front of a marker (along marker +Z).
APPROACH_DISTANCE = 0.80  # meters

# ArUco PnP accuracy degrades rapidly with distance. For a 25 cm marker
# in a 640x480 image:
#   ~4.0 m -> ~45 px wide, ~30 cm PnP error (used to be our value)
#   ~3.0 m -> ~60 px wide, ~15-20 cm PnP error
#   ~2.5 m -> ~75 px wide, ~10-15 cm PnP error
# With APPROACH_DISTANCE lowered to 0.40 m for task 3 pick-and-place,
# the marker_in_map position needs to be accurate to ~15 cm so the
# approach pose lands inside the inflation-free corridor in front of
# the cube. 3.0 m is the sweet spot: enough accuracy for the closer
# approach without forcing the robot to crawl right up to every wall
# during the random search.
MAX_DETECTION_DISTANCE = 3.5  # meters
