# TASK 3 of Autonomous Mobile Robotics Exam - Group 30
#
# Service-client wrapper around the IFRA gazebo_ros_link_attacher plugin
# (loaded by the tiago_exam Gazebo world). The plugin advertises two
# services:
#
#    /ATTACHLINK   (linkattacher_msgs/srv/AttachLink)
#    /DETACHLINK   (linkattacher_msgs/srv/DetachLink)
#
# Both share the same request fields:
#   string model1_name, link1_name, model2_name, link2_name
# and a (success: bool, message: string) response.
#
# We attach by (tiago, gripper_link) <-> (aruco_cube_exam_id<N>, link).
#
# IMPORTANT: per the exam spec the use of "non-standard / questionable
# procedures, e.g. calling ROS topic or services as subprocesses within
# another node" is penalized. We therefore use rclpy's native service-
# client API: create_client(...) + call_async(...). No subprocess, no
# shelling out to `ros2 service call`.

from linkattacher_msgs.srv import AttachLink, DetachLink

from tiago_project_group30.task3_constants import (
    ATTACH_LINK_SERVICE,
    CUBE_LINK_NAME,
    CUBE_MODEL_NAMES,
    DETACH_LINK_SERVICE,
    TIAGO_GRIPPER_LINK,
    TIAGO_MODEL_NAME,
)


class LinkAttacher:

    def __init__(self, node, callback_group):
        self.node = node
        self._attach_client = node.create_client(
            AttachLink, ATTACH_LINK_SERVICE, callback_group=callback_group
        )
        self._detach_client = node.create_client(
            DetachLink, DETACH_LINK_SERVICE, callback_group=callback_group
        )
        # State machine polls these instead of awaiting the future.
        self._pending_future = None
        self.action_done = False
        self.action_succeeded = False
        self.last_message = ""

    # ------------------------------------------------------------------
    # Issue requests
    # ------------------------------------------------------------------
    def attach(self, cube_id: int):
        # Wait synchronously for the service to be available -- the plugin
        # is loaded with Gazebo so it's up by the time Task 3 starts.
        if not self._attach_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error(
                f"{ATTACH_LINK_SERVICE} not available"
            )
            self.action_done = True
            self.action_succeeded = False
            self.last_message = "service unavailable"
            return
        req = AttachLink.Request()
        req.model1_name = TIAGO_MODEL_NAME
        req.link1_name = TIAGO_GRIPPER_LINK
        req.model2_name = CUBE_MODEL_NAMES[cube_id]
        req.link2_name = CUBE_LINK_NAME
        self.action_done = False
        self.action_succeeded = False
        self.last_message = ""
        self.node.get_logger().info(
            f"Link attacher: ATTACH "
            f"({req.model1_name}.{req.link1_name}) <-> "
            f"({req.model2_name}.{req.link2_name})"
        )
        self._pending_future = self._attach_client.call_async(req)
        self._pending_future.add_done_callback(self._on_future_done)

    def detach(self, cube_id: int):
        if not self._detach_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error(
                f"{DETACH_LINK_SERVICE} not available"
            )
            self.action_done = True
            self.action_succeeded = False
            self.last_message = "service unavailable"
            return
        req = DetachLink.Request()
        req.model1_name = TIAGO_MODEL_NAME
        req.link1_name = TIAGO_GRIPPER_LINK
        req.model2_name = CUBE_MODEL_NAMES[cube_id]
        req.link2_name = CUBE_LINK_NAME
        self.action_done = False
        self.action_succeeded = False
        self.last_message = ""
        self.node.get_logger().info(
            f"Link attacher: DETACH "
            f"({req.model1_name}.{req.link1_name}) <-> "
            f"({req.model2_name}.{req.link2_name})"
        )
        self._pending_future = self._detach_client.call_async(req)
        self._pending_future.add_done_callback(self._on_future_done)

    # ------------------------------------------------------------------
    # Future completion -> set flags
    # ------------------------------------------------------------------
    def _on_future_done(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.node.get_logger().error(f"Link attacher service raised: {e}")
            self.action_done = True
            self.action_succeeded = False
            self.last_message = str(e)
            return
        self.action_succeeded = bool(resp.success)
        self.last_message = resp.message
        if self.action_succeeded:
            self.node.get_logger().info(f"Link attacher OK: {resp.message}")
        else:
            self.node.get_logger().warn(f"Link attacher FAILED: {resp.message}")
        self.action_done = True
