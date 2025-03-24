from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ParrotCameraNode(Node):
    def __init__(self):
        super().__init__('parrot_camera_node')
        self.bridge = CvBridge()
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            Image,
            '/bebop/camera/image_raw',
            self.image_callback,
            qos_profile)
        
        self.get_logger().info('Subscribed to /bebop/image_raw')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Bebop Camera View", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

def main(args=None):
    rclpy.init(args=args)
    viewer = ParrotCameraNode()
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()