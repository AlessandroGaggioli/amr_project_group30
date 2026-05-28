import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import PyKDL as kdl
import numpy as np

class FramePublisher(Node):

    def __init__(self):
        super().__init__('example_tf_broadcaster')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.parent_name = "base_footprint"
        self.child_name = "my_new_frame"

        # Create a timer to call publish_frame periodically
        self.timer = self.create_timer(0.1, self.publish_frame)  # 0.1 seconds = 10 Hz


    def publish_frame(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_name
        t.child_frame_id = self.child_name

        # position
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.0

        rot = kdl.Rotation()
        rot.DoRotX(np.pi/4)

        quat = rot.GetQuaternion()
        
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
