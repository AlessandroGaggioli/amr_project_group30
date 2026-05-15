import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped

from PyKDL import Frame, Vector, Rotation

class ArucoSubscriber(Node):

    def __init__(self):
        super().__init__('aruco_subscriber')
        
        self.subscription = self.create_subscription(TransformStamped, '/aruco_single/transform', self.callback, 1)

    def callback(self, msg):
        
        position = Vector(msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z)
        orientation = Rotation.Quaternion(msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w) 

        frame = Frame(orientation, position)
        

def main(args=None):
    rclpy.init(args=args)

    sub = ArucoSubscriber()

    rclpy.spin(sub)
    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()