# Autonomous Mobile Robotics Exam - Group 30
#
# Task 3 - Cube link attacher

# Uses Service Client to request attach/detach operations.


from linkattacher_msgs.srv import AttachLink, DetachLink

from tiago_project_group30.constants import (
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
        # Create the ROS 2 service clients that will talk to the Gazebo plugin
        self._attach_client = node.create_client(
            AttachLink, ATTACH_LINK_SERVICE, callback_group=callback_group
        )
        self._detach_client = node.create_client(
            DetachLink, DETACH_LINK_SERVICE, callback_group=callback_group
        )

        # Status flags monitored by the State Machine
        self._pending_future = None
        self.action_done = False
        self.action_succeeded = False
        self.last_message = ""

    # ---------------
    # Issue requests
    # ---------------
    def attach(self, cube_id: int):
        # Wait synchronously for the service to be available -- the plugin
        # is loaded with Gazebo so it's up by the time Task 3 starts.

        # Check if the Gazebo plugin is actually running and listening
        if not self._attach_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error(
                f"{ATTACH_LINK_SERVICE} not available"
            )
            self.action_done = True
            self.action_succeeded = False
            self.last_message = "service unavailable"
            return
        
        # Build the request telling Gazebo exactly which two parts to connect
        req = AttachLink.Request()
        req.model1_name = TIAGO_MODEL_NAME
        req.link1_name = TIAGO_GRIPPER_LINK
        req.model2_name = CUBE_MODEL_NAMES[cube_id]
        req.link2_name = CUBE_LINK_NAME

        # Reset the status flags before sending the new request
        self.action_done = False
        self.action_succeeded = False
        self.last_message = ""
        
        self.node.get_logger().info(
            f"Link attacher: ATTACH "
            f"({req.model1_name}.{req.link1_name}) <-> "
            f"({req.model2_name}.{req.link2_name})"
        )

        # Send the request asynchronously
        self._pending_future = self._attach_client.call_async(req)
        self._pending_future.add_done_callback(self._on_future_done)

    def detach(self, cube_id: int): # same as attach, just different service 
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

    # -----------
    # Set flags 
    # -----------
    def _on_future_done(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.node.get_logger().error(f"Link attacher service raised: {e}")
            self.action_done = True
            self.action_succeeded = False
            self.last_message = str(e)
            return
        
        # Update our trackers based on Gazebo's success/failure report
        self.action_succeeded = bool(resp.success)
        self.last_message = resp.message

        if self.action_succeeded:
            self.node.get_logger().info(f"Link attacher OK: {resp.message}")
        else:
            self.node.get_logger().warn(f"Link attacher FAILED: {resp.message}")

        # Finally, tell the State Machine that the operation is finished
        self.action_done = True
