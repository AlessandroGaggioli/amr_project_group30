import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TfListener(Node):

    def __init__(self):
        super().__init__("tf_listener")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.timer_tf)

        self.frame1 = "base_link"
        self.frame2 = "arm_tool_link"

    def timer_tf(self):
        try:
            self.tf = self.tf_buffer.lookup_transform(self.frame1, self.frame2, rclpy.time.Time())
            print(self.tf)
        except:
            self.get_logger().info("Could not transform!")
        return


def main(args=None):
    rclpy.init(args=args)

    pp = TfListener()

    rclpy.spin(pp)
    pp.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
