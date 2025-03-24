from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image , CompressedImage
from cv_bridge import CvBridge
import cv2

class ParrotCompressCameraNode(Node):
    def __init__(self):
        super().__init__('compression_node')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/bebop/image_raw',
            self.image_callback,
            10)
        
        self.publisher = self.create_publisher(
            CompressedImage,
            '/bebop/camera/image_compressed',
            10)
        
        self.get_logger().info('Subscribed to /bebop/camera/image_raw')

    def image_callback(self, msg):

        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Compress image using JPEG
            success, encoded_image = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 20])
            if not success:
                self.get_logger().warn("Failed to compress image.")
                return

            # Create and publish CompressedImage message
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = 'jpeg'
            compressed_msg.data = encoded_image.tobytes()

            self.publisher.publish(compressed_msg)

        except Exception as e:
            self.get_logger().error(f"Compression error: {e}")

def main(args=None):
    rclpy.init(args=args)
    viewer = ParrotCompressCameraNode()
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
