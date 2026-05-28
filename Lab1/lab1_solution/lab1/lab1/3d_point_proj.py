import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.wait_for_message import wait_for_message

import numpy as np
from scipy.spatial.transform import Rotation


class PointProjection(Node):

    def __init__(self):
        super().__init__('image_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.timer_tf_camera)

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.callback, 1)
        self.camera_pub = self.create_publisher(Image, '/head_front_camera/rgb/image_raw_point_proj', 1)
        self.camera_info_topic = "/head_front_camera/rgb/camera_info"

        self.t_gripper_frame_to_camera = None
        self.camera_frame = "head_front_camera_rgb_optical_frame"
        self.gripper_frame = "gripper_grasping_frame"

        self.cam_K = None
        self.cam_D = None

        self.point_pub = self.create_publisher(Point, "target_point", 1)


    def timer_tf_camera(self):
        try:
            self.t_gripper_frame_to_camera = self.tf_buffer.lookup_transform(self.camera_frame, self.gripper_frame, rclpy.time.Time())
        except:
            self.get_logger().info('Could not transform camera!')
        return
      

    def homogeneous_matrix_from_transform(self, t):
        position = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        quat = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        T_matrix = np.eye(4)
        T_matrix[:3, 3] = position
        T_matrix[:3, :3] = Rotation.from_quat(quat).as_matrix()
        return T_matrix


    def project(self, points3d, camera_pose, cam_K, cam_D):
        R, _ = cv2.Rodrigues(camera_pose[:3, :3])
        t = camera_pose[:3, 3]
        points3d = points3d.reshape(-1, 3).astype(np.float32)
        points2d, _ = cv2.projectPoints(points3d, R, t, cam_K, cam_D)
        return points2d.squeeze()

    def callback(self, msg):
        
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if self.cam_K is None:
            ret, msg =  wait_for_message(CameraInfo, self, self.camera_info_topic)
            self.cam_K = np.array(msg.k).reshape(3,3)
            self.cam_D = np.array(list(msg.d))

        if self.t_gripper_frame_to_camera is not None:
            camera_pose = self.homogeneous_matrix_from_transform(self.t_gripper_frame_to_camera)

            print("translation part: ", camera_pose[:3, 3])

            position = np.array([0, 0, 0])
            point2d = self.project(position, camera_pose, self.cam_K, self.cam_D)

            out_img = self.img.copy()
            cv2.circle(out_img, (int(point2d[0]), int(point2d[1])), 10, (0, 255, 0), -1)

            out_msg = self.bridge.cv2_to_imgmsg(out_img, encoding="bgr8")
            self.camera_pub.publish(out_msg)

            # publish point2d
            point_msg = Point(x=float(point2d[0]), y=float(point2d[1]))
            self.point_pub.publish(point_msg)


            cv2.imshow("image", out_img)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    pp = PointProjection()

    rclpy.spin(pp)
    pp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()