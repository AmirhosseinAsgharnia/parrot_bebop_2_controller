import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

class EmergencyLand(Node):

    def __init__(self):    
        super().__init__("emergency_land")

        self.get_logger().info("Emergency Land Activated")
        self.pulisher_land     = self.create_publisher(Empty , "/bebop/land" , 10)
        self.pulisher_cmd_vel  = self.create_publisher(Twist , "/bebop/cmd_vel" , 10)
        self.land()
        
    def land(self):
        cmd_stop = Twist()  # Zero all velocities
        self.pulisher_cmd_vel.publish(cmd_stop)
        self.get_logger().info("Published zero velocity before landing.")

        time.sleep(0.1)  # Let it stabilize

        cmd_land = Empty()

        for i in range(5):  # Retry publishing
            if self.pulisher_land.get_subscription_count() > 0:
                self.pulisher_land.publish(cmd_land)
                self.get_logger().info(f"Landing command attempt {i+1}")
            else:
                self.get_logger().warn("No subscriber on /bebop/land")
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyLand()
    rclpy.spin(node) 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()