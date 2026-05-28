import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self.callback,
            1)

        self.bridge = CvBridge()

    def callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        cv2.imshow("Camera Image", self.img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    image_sub = ImageSubscriber()

    rclpy.spin(image_sub)
    image_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()