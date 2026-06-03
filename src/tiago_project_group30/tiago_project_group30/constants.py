# Autonomous Mobile Robotics Exam - Group 30
#

# timeouts for arm movement
EXECUTION_TIMEOUT = 30.0  # max time to finish once EXECUTING
PLANNING_TIMEOUT = 6.0 # Time allowed for MoveIt to plan an arm path
EXECUTION_TIMEOUT = 30.0  # Time allowed for the physical arm movement
SEARCH_NAV_TIMEOUT = 120.0 # Time allowed to reach a random exploration point
APPROACH_NAV_TIMEOUT = 180.0 # Time allowed to reach the pick/place approach pose.
ARM_MOTION_TIMEOUT = 20.0   # MoveIt2 plan+execute per arm goal
GRIPPER_TIMEOUT = 5.0
ATTACH_TIMEOUT = 3.0
CUBE_DETECTION_TIMEOUT = 15.0   # max wait for cube marker after head tilt

# ------------------
# Tiago parameters
# ------------------

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

HOME_JOINT_POSITIONS = [0.15, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]

# -------------------
# Search strategy
# -------------------

# When sampling a new waypoint we reject any (x, y) that is
# within this distance of a previously-tried waypoint.
VISITED_RADIUS = 1.5  # meters

# Sampling is biased toward cells close to the robot's current position.
# - PREFERRED_RANGE: cells beyond this get exponentially less likely.
# - SOFT_MAX_RANGE: cells beyond this are excluded entirely unless no closer cell is available. 
# The robot first surveys nearby cells. 
PREFERRED_RANGE = 2.0  # meters
SOFT_MAX_RANGE = 3.0   # meters

# Don't sample cells closer than this many meters to the saved-map edge.
EDGE_MARGIN = 1.0  # meters

# ----------------------------
# ArUco / approach parameters
# ----------------------------

PICK_MARKER_FRAME_RAW = "aruco_pick_frame" # published by aruco_single ID 26
PLACE_MARKER_FRAME_RAW = "aruco_place_frame" # published by aruco_single ID 238

# Derived approach TFs broadcasted 
PICK_APPROACH_FRAME = "aruco_pick_approach"
PLACE_APPROACH_FRAME = "aruco_place_approach"

# Camera optical frame
CAMERA_FRAME = "head_front_camera_rgb_optical_frame"

# Distance the robot stops in front of a marker. 
APPROACH_DISTANCE = 0.80  # meters

# Maximum distance at which the ArUco tracker latches a marker and broadcasts the approach frames.
MAX_DETECTION_DISTANCE = 4.0  # meters

# ------------------------
# Per-waypoint head sweep 
# ------------------------
SEARCH_PAN_POSITIONS = [-0.6, 0.6, 0.0]   # rad — left, right, centre
SEARCH_PAN_DWELL = 1.5                    # seconds per pan position

# -------------------
# ArUco cube markers
# -------------------
# ID 63 first, then ID 582.
CUBE_PICK_SEQUENCE = [63, 582]

CUBE_ARUCO_TOPICS = {
    63:  "/aruco_cube_63/aruco_single/transform",
    582: "/aruco_cube_582/aruco_single/transform",
}

CUBE_ARUCO_FRAMES = {
    63:  "aruco_cube_63_frame",
    582: "aruco_cube_582_frame",
}

CUBE_MODEL_NAMES = {
    63:  "aruco_cube_exam_id63",
    582: "aruco_cube_exam_id582",
}

CUBE_LINK_NAME = "link"

# Physical cube side 
CUBE_SIDE = 0.07  # meters
# Marker is on top face so the marker frame sits CUBE_SIDE/2 
CUBE_TOP_TO_CENTER = CUBE_SIDE / 2.0  # 0.035 m

CUBE_APPROACH_DISTANCE = 0.75  
DRIVE_SPEED = 0.15          
SAFE_NAV_DISTANCE = 1.6   

# -------
# Tiago 
# -------
TIAGO_MODEL_NAME = "tiago"
TIAGO_GRIPPER_LINK = "gripper_left_finger_link"
# ---------
# Gripper 
# ---------
GRIPPER_JOINT_NAMES = ["gripper_left_finger_joint", "gripper_right_finger_joint"]

GRIPPER_OPEN_POSITIONS = [0.045, 0.045]
GRIPPER_CLOSED_POSITIONS = [0.037, 0.037]

GRIPPER_GROUP_NAME = "gripper"
GRIPPER_COMMAND_ACTION_NAME = "gripper_controller/joint_trajectory"

# ---------------
# Link attacher 
# ---------------
ATTACH_LINK_SERVICE = "/ATTACHLINK"
DETACH_LINK_SERVICE = "/DETACHLINK"
# -----------------------
# Grasp / place geometry
# -----------------------

GRASP_Z_ABOVE_TOP = 0.04   # meters above the cube top face
PRE_GRASP_LIFT = 0.40   # meters above the cube CENTER (increased from 0.10)

# Post-grasp / carry: lift the cube up before navigating away.
POST_GRASP_LIFT = 0.30  # meters above the cube CENTER
PLACE_SURFACE_TOP_Z = 0.30      # meters 
PLACE_DROP_CLEARANCE = 0.02     # safety margin so we don't slam the surface

PLACE_TARGET_Z = (
    PLACE_SURFACE_TOP_Z + PLACE_DROP_CLEARANCE + CUBE_SIDE + GRASP_Z_ABOVE_TOP
)

# When placing, Nav2 parks the base this far from the place wall marker. 
PLACE_FORWARD_OFFSET = 0.65   # meters
PLACE_APPROACH_DISTANCE = 0.40  # meters between base_link and place wall marker
PLACE_LATERAL_OFFSET = 0.15  # meters

# -----------
# Head tilt
# -----------
HEAD_TILT_DOWN_FOR_CUBE = -1.0   # rad

HEAD_SCAN_PANS = [0.0, -1.3, 1.3]   # rad
HEAD_SCAN_DWELL = 3.0   # seconds per pan position

