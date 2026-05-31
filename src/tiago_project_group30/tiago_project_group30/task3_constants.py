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
# Close to FLAT CONTACT on the 7 cm cube, not all the way to 0. At
# position 0.044 the opening is ~8 cm, so each finger pad sits ~position*
# 0.909 from the cube centre; flat contact on a 7 cm cube (pad at 3.5 cm)
# is ~0.0385. We command 0.037 -> ~1.5 mm total preload: the pads press
# flat on two faces and self-centre the cube without squeezing it.
# Earlier values were wrong in OPPOSITE ways: 0/0 drove the pads ~7 cm PAST
# the faces (cube squirts out), and 0.032 over-squeezed the rigid cube by
# ~1.2 cm, generating large contact forces that SPUN the cube crooked an
# instant before the IFRA attach froze it. The attach does the real
# holding, so the fingers only need gentle flat contact.
GRIPPER_CLOSED_POSITIONS = [0.037, 0.037]
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
# Height of the gripper_grasping_frame above the cube's TOP face at the
# grasp pose. The Tiago fingers are long (~0.10 m of fingertip BELOW the
# grasping_frame along the approach axis), so a top-down grasp aimed at the
# cube center drove the fingertips below the cube base and into the table
# top. Raising the target so the grasping_frame sits this far ABOVE the
# cube top keeps the fingertips around the cube's upper body instead of
# punching through the table. Tune to taste (cube is 7 cm tall).
GRASP_Z_ABOVE_TOP = 0.04   # meters above the cube top face

# Pre-grasp = same XY as cube, lifted vertically by this much (arm hovers
# above, then descends straight down to the grasp pose):
PRE_GRASP_LIFT = 0.25   # meters above the cube CENTER (increased from 0.10)

# Yaw tweak added to the cube marker's own yaw when orienting the gripper.
# The finger opening axis (grasping_frame Y) is set parallel to a pair of
# cube faces via the marker yaw. At offset 0 the fingers came down ALIGNED
# WITH THE ROBOT'S X AXIS (the approach line), so one finger descended onto
# the near face and rammed the cube. pi/2 rotates the opening axis 90 deg so
# the two fingers straddle the LEFT/RIGHT faces and the cube drops into the
# gap between them.
CUBE_GRASP_YAW_OFFSET = 1.5707963267948966   # radians (pi/2)

# Post-grasp / carry: lift the cube up before navigating away.
POST_GRASP_LIFT = 0.20  # meters above the cube CENTER

# Place-side geometry: the cube is dropped on the place surface. The place
# surface top (env_exam_place_surface) is at z ~ 0.30 m.
PLACE_SURFACE_TOP_Z = 0.30      # meters (env_exam_place_surface top)
PLACE_DROP_CLEARANCE = 0.02     # safety margin so we don't slam the surface
# PLACE_TARGET_Z is the gripper_grasping_frame height (map Z) at the drop.
# It MUST be consistent with how the grasp holds the cube: at grasp time the
# grasping_frame sits GRASP_Z_ABOVE_TOP above the cube top face, so the cube
# center is (GRASP_Z_ABOVE_TOP + CUBE_TOP_TO_CENTER) BELOW the grasping_frame.
# To rest the cube bottom on the surface (+ clearance):
#   grasping_frame_z = surface + clearance + CUBE_SIDE + GRASP_Z_ABOVE_TOP
# The old value (surface + 0.04 + clearance = 0.36) was tuned for a DIFFERENT
# grasp offset; at 0.36 the fingertips (~0.10 m below) and the held cube ended
# up BELOW the 0.30 m surface top, so the DROP goal collided with the table
# and OMPL reported "Unable to sample any valid states for goal tree".
PLACE_TARGET_Z = (
    PLACE_SURFACE_TOP_Z + PLACE_DROP_CLEARANCE + CUBE_SIDE + GRASP_Z_ABOVE_TOP
)

# Forward offset from the robot's CURRENT base (see _precompute_drop_poses)
# where the cube is set down. With the base ~PLACE_APPROACH_DISTANCE (0.60 m)
# from the place wall after the push, 0.55 m forward lands the cube ~5 cm
# shy of the wall -- on the surface. 0.45 m left it ~15 cm from the wall,
# just short of the surface edge (cube dropped in front of the table).
# Still inside arm reach (the pick worked at ~0.68 m forward).
PLACE_FORWARD_OFFSET = 0.65    # meters

# Like the pick side, Nav2 parks short of the place wall because of
# inflation, leaving the robot too far to set the cube on the surface.
# After State 5 reaches the PLACE approach, State 27 (in "place" mode)
# pushes the base toward the PLACE WALL MARKER (aruco.place_marker_in_map),
# steering frontal, until base_link is within this distance of it -- the
# same closed-loop engine used for the pick cube approach.
PLACE_APPROACH_DISTANCE = 0.65  # meters between base_link and place wall marker

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
# Cube approach (refined Nav2 goal + closed-loop /cmd_vel push)
##############################
# After the first cube detection we navigate (NavigateToPose) to a pose
# SAFE_NAV_DISTANCE m in front of the cube. Nav2 obeys inflation_radius
# (0.7 m around the pick surface) and cannot park closer than ~1 m -- so
# we close the remaining gap with a closed-loop /cmd_vel push (State 27):
# the state machine drives the base forward while watching the
# map->base_link TF and stops the instant base_link is within
# CUBE_APPROACH_DISTANCE m of the cube center. That value is chosen so the
# cube falls inside Tiago's arm workspace (~0.92 m max horizontal reach at
# the cube's height of 0.34 m).
# Sweet spot between two failure modes, now that State 27 steers the base
# FRONTAL to the cube (no lateral offset):
#   - too close (<0.65) -> the top-down arm sweeps its elbow into the front
#     edge of the pick table while descending.
#   - too far  (0.75)   -> the centered cube lands ~0.82 m forward, at the
#     edge of the arm's forward reach, and OMPL can't sample a valid
#     pre-grasp ("Unable to sample any valid states for goal tree").
# 0.65 m puts the cube ~0.72 m forward: clear of the table edge AND
# comfortably inside Tiago's ~0.92 m max horizontal reach. The earlier
# table strike at 0.65 was caused by the SKEWED approach (cube off to the
# side -> arm reaching sideways), which the State 27 steering term fixes.
CUBE_APPROACH_DISTANCE = 0.70   # meters between base_link and cube center XY
DRIVE_SPEED = 0.15          # m/s for the /nav_vel forward push (0.05 was far
                            # too timid: the smoother/mux ramps it down, so the
                            # base crawled ~0.016 m/s and never closed the gap)
SAFE_NAV_DISTANCE = 1.1   # Nav2 goal is this far from the cube, then we push the last bit