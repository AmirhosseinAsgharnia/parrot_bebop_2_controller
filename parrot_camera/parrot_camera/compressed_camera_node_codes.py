import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import cv2

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_camera_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            '/bebop/camera/image_compressed',
            self.image_callback,
            qos_profile)
        
        self.get_logger().info("Subscribed to /bebop/image_compressed")

    def image_callback(self, msg):
        try:
            # Convert byte data to NumPy array
            np_arr = np.frombuffer(msg.data, np.uint8)

            # Decode compressed image (e.g., JPEG)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is not None:
                cv2.imshow("Decompressed Image", cv_image)
                cv2.waitKey(1)
            else:
                self.get_logger().warn("Failed to decode image.")
        except Exception as e:
            self.get_logger().error(f"Error decoding image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
