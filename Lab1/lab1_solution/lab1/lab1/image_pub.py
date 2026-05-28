import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')

        self.bridge = CvBridge()
        self.camera_pub = self.create_publisher(Image, '/head_front_camera/rgb/image_raw_flipped', 1)

        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self.callback,
            1)

    def callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        img_flipped = cv2.flip(self.img, 0) # just for example
        img_msg = self.bridge.cv2_to_imgmsg(img_flipped, encoding="bgr8")
        self.camera_pub.publish(img_msg)

        cv2.imshow("image", img_flipped)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    image_pub = ImagePublisher()

    rclpy.spin(image_pub)
    image_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()