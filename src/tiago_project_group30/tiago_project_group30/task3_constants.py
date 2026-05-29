# TASK 3 of Autonomous Mobile Robotics Exam - Group 30
#
# Constants for the pick-and-place sub-flow that runs AFTER Task 2 has
# delivered the robot to the PICK and PLACE approach poses.
#
# Sources:
#   - Gripper joints / open-close positions: Lab 2 basic_gripper.py
#   - Cube model names / link names: tiago_exam_worlds SDFs
#       aruco_cube_exam_id63/aruco_cube_exam_id63.sdf
#       aruco_cube_exam_id582/aruco_cube_exam_id582.sdf
#     Both cubes share <link name="link"> and the ArUco marker is mounted on
#     the +Z face (pose 0 0 0.035 0 0 0 inside the model -> marker on top).
#   - Tiago model name + gripper wrist link from pal_gripper_description.

##############################
# ArUco cube markers
##############################
# Sequence imposed by the exam: ID 63 first, then ID 582.
CUBE_PICK_SEQUENCE = [63, 582]

# Topic names produced by the aruco_single nodes spawned in task3.launch.py.
# Pattern: /<namespace>/aruco_single/transform   (reference_frame='')
CUBE_ARUCO_TOPICS = {
    63:  "/aruco_cube_63/aruco_single/transform",
    582: "/aruco_cube_582/aruco_single/transform",
}
# TF frames the same aruco_single nodes broadcast (marker_frame param).
CUBE_ARUCO_FRAMES = {
    63:  "aruco_cube_63_frame",
    582: "aruco_cube_582_frame",
}
# Gazebo model names (must match the SDF <model name=...>).
CUBE_MODEL_NAMES = {
    63:  "aruco_cube_exam_id63",
    582: "aruco_cube_exam_id582",
}
# All cubes use <link name="link"> (see the SDFs).
CUBE_LINK_NAME = "link"

# Physical cube side (exam spec: 7 cm).
CUBE_SIDE = 0.07  # meters
# Marker is on top face -> marker frame sits CUBE_SIDE/2 above cube center.
CUBE_TOP_TO_CENTER = CUBE_SIDE / 2.0  # 0.035 m

##############################
# Tiago side
##############################
# Tiago is spawned in Gazebo with model name 'tiago'.
TIAGO_MODEL_NAME = "tiago"
# The wrist link the gripper is rigidly bolted to (pal_gripper_description).
# Attaching the cube to this link makes it follow the gripper as a rigid body
# while not interfering with the prismatic finger joints.
TIAGO_GRIPPER_LINK = "gripper_left_finger_link"

##############################
# Gripper interface (Lab 2 / pymoveit2)
##############################
GRIPPER_JOINT_NAMES = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
GRIPPER_OPEN_POSITIONS = [0.044, 0.044]   # 8 cm total opening, > 7 cm cube
GRIPPER_CLOSED_POSITIONS = [0.0, 0.0]
GRIPPER_GROUP_NAME = "gripper"
GRIPPER_COMMAND_ACTION_NAME = "gripper_controller/joint_trajectory"

##############################
# Link attacher services
##############################
ATTACH_LINK_SERVICE = "/ATTACHLINK"
DETACH_LINK_SERVICE = "/DETACHLINK"

##############################
# Grasp / place geometry
##############################
# Top-down grasp (Lab 3 "modify approach_frame to grasp from top of marker"
# hint). The cube marker frame has Z=up (marker on TOP face of the cube),
# so DoRotY(pi/2) on it -- the standard Lab 3 approach modifier -- gives:
#   orientation RPY(0, pi/2, yaw) in map frame →
#     X_grasping: straight down (approach axis: arm descends onto cube)
#     Y_grasping: horizontal perpendicular
#     Z_grasping: horizontal along yaw = gripper_link Y = finger opening axis
#   → fingers open along the line between robot and cube, both at cube
#     center height, clear of the support surface.
# The MoveIt2 end-effector is `gripper_grasping_frame` (tiago_arm.py).
#
# Pre-grasp = same XY as cube, lifted vertically by this much (arm hovers
# above, then descends straight down to the grasp pose):
PRE_GRASP_LIFT = 0.20   # meters above the cube CENTER

# Post-grasp / carry: lift the cube up before navigating away.
POST_GRASP_LIFT = 0.20  # meters above the cube CENTER

# Place-side geometry: the cube is dropped on the place surface. The place
# surface top (env_exam_place_surface) is at z ~ 0.30 m, so dropping the
# cube CENTER at z = 0.30 + cube_half + small clearance lands it cleanly.
PLACE_SURFACE_TOP_Z = 0.30      # meters (env_exam_place_surface top)
PLACE_DROP_CLEARANCE = 0.02     # safety margin so we don't slam the surface
# Since we grasp at cube_top - 0.01 (0.04 from bottom), the gripper grasping frame
# must be at Z = surface + 0.04 + clearance for drop.
PLACE_TARGET_Z = PLACE_SURFACE_TOP_Z + 0.04 + PLACE_DROP_CLEARANCE

# Forward offset in front of the PLACE approach pose where the cube is set
# down (measured from the robot toward the marker). The robot stops at
# APPROACH_DISTANCE = 0.55 m in front of the place wall, so 0.45 m forward
# lands the cube ~10 cm shy of the wall -- on the surface, not pressed
# against it.
PLACE_FORWARD_OFFSET = 0.45    # meters

##############################
# Head tilt
##############################
# Task 2 uses -0.5 rad (~17 deg) which is enough to keep wall markers in
# view. For cubes on a ~0.3 m-tall surface seen from APPROACH_DISTANCE
# = 0.55 m with the camera at ~1.5 m, we need a much steeper tilt:
#   theta ~ atan( (1.5 - 0.37) / 0.55 ) ~ 1.1 rad
HEAD_TILT_DOWN_FOR_CUBE = -1.0   # rad, mild over-tilt for safety

# Active head scan used in State 11. When the cube is not visible from
# the PICK approach pose (e.g. it is slightly off-axis from the wall
# marker), we sweep the head through these pan positions until the
# cube marker appears in the camera FOV. ~35 deg each side covers the
# whole surface in front of the wall.
HEAD_SCAN_PANS = [0.0, -1.3, 1.3]   # rad
HEAD_SCAN_DWELL = 3.0                          # seconds per pan position

##############################
# Cube approach (refined Nav2 goal + drive_on_heading push)
##############################
# After the first cube detection we navigate to a pose at this distance
# in front of the cube and pointing at it. Nav2's NavigateToPose obeys
# inflation_radius (0.7 m around the pick surface) and therefore cannot
# park closer than ~1 m to the cube -- so the planner stops short, and
# we close the remaining gap using Nav2's behavior action
# /drive_on_heading (which ignores the costmap) until base_link is
# actually CUBE_APPROACH_DISTANCE m from the cube center. The value is
# chosen so the cube falls inside Tiago's arm workspace (~0.92 m max
# horizontal reach at the cube's height of 0.34 m).
CUBE_APPROACH_DISTANCE = 0.85   # meters between base_link and cube center XY
DRIVE_SPEED = 0.05          # m/s for the drive_on_heading push
