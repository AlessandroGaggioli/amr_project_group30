import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from rclpy.wait_for_message import wait_for_message
import numpy as np


class PointProjection2(Node):

    def __init__(self):
        super().__init__('from_2d_point_to_3d')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/head_front_camera/depth_registered/image_raw', self.callback, 1)
        self.point_sub = self.create_subscription(Point, '/target_point', self.callback_point, 1)
        self.camera_info_topic = "/head_front_camera/rgb/camera_info"

        self.cam_K = None
        self.cam_D = None
        self.point = None


    def callback_point(self, msg):
        self.point = [msg.x, msg.y]

    def callback(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)

        if self.cam_K is None:
            ret, info_msg =  wait_for_message(CameraInfo, self, self.camera_info_topic)
            self.cam_K = np.array(info_msg.k).reshape(3,3)
            self.cam_D = np.array(list(info_msg.d))

        if self.cam_K is not None and self.depth_img is not None:
            print(self.point)
            depth_value = self.depth_img[int(self.point[1]), int(self.point[0])]

            X = depth_value * (self.point[0] - self.cam_K[0,2]) / self.cam_K[0,0]
            Y = depth_value * (self.point[1] - self.cam_K[1,2]) / self.cam_K[1,1]
            Z = depth_value

            print(X, Y, Z)

def main(args=None):
    rclpy.init(args=args)

    pp = PointProjection2()

    rclpy.spin(pp)
    pp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()