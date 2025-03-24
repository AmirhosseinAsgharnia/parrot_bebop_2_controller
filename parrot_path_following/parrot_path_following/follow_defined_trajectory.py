
import math
import time
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Empty
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation
from mocap_optitrack_interfaces.msg import RigidBodyArray
import subprocess

class PathFollowing(Node): # MyNode: the node class. For example CameraNode, SensorNode
    
    def __init__(self):
        
        super().__init__("follow_path")
        self.get_logger().info("follow_path node has been established!")

        # Establish publishers
        self.pulisher_take_off = self.create_publisher(Empty , "/bebop/takeoff" , 10)
        self.pulisher_land     = self.create_publisher(Empty , "/bebop/land" , 10)
        self.pulisher_cmd_vel  = self.create_publisher(Twist , "/bebop/cmd_vel" , 10)
        self.subscriber_OT_camera = self.create_subscription(RigidBodyArray , "/mocap_rigid_bodies" , self.ot_camera_feedback , 10)
        
        #self.phase_reader = self.create_subscription(Int32 , "/mission_phase" , self.phase_reader , 10)
        
        time.sleep(5)
        self.phase_timer = self.create_timer(0.05, self.phase_manual)

        # Take-off command
        
        self.take_off_cmd = False

        #self.timer = self.create_timer(0.05, self.path_controller)
        self.counter = 0
        self.get_logger().info("Publishers and Subscribers are up.")

        # Parameters
        self.robot_id = 5

        # Path PID Controller
        self.P_x = self.P_y = 0.5
        self.I_x = self.I_y = 0.0
        self.D_x = self.D_y = 0.2

        self.P_z = 0.4
        self.I_z = 0.0
        self.D_z = 0.2

        self.P_yaw = 0.4
        self.I_yaw = 0.0
        self.D_yaw = 0.2

        self.integral_x   = self.integral_y   = self.integral_z   = self.prev_error_x = self.prev_error_y = self.prev_error_z = 0
        self.integral_yaw = self.prev_error_yaw = 0

        self.initial_x = 0
        self.initial_y = 0
        self.init_state = True
    def phase_reader(self , phase_msg):
        self.phase = phase_msg.data

        self.goal_x = 0
        self.goal_y = 0
        self.goal_z = 0.5

        if self.counter <100:
            self.desired_yaw = 0
        else:
            self.desired_yaw = 0

    def phase_manual(self):

        if self.counter == 0:
            
            self.take_off()
            time.sleep(5)
            phase = 1
        elif self.counter < 200:
            
            V_x = ( 0 - self.initial_x  ) / 10
            V_y = ( 1.5 - self.initial_y  ) / 10

            self.goal_x = V_x * self.counter * 0.05 + self.initial_x
            self.goal_y = V_y * self.counter * 0.05 + self.initial_y
            self.goal_z = 1.0
            self.desired_yaw = math.pi
            self.path_controller()

            if self.counter == 200 - 1:
                self.init_state = True
            phase = 2
        elif self.counter < 300:
            V_yaw = ( 0 - self.initial_yaw  ) / 5

            self.goal_x = self.initial_x
            self.goal_y = self.initial_y
            self.goal_z = 1.0
            self.desired_yaw = V_yaw * ( self.counter - 200 ) * 0.05 + self.initial_yaw
            self.path_controller()

            if self.counter == 300 - 1:
                self.init_state = True
            phase = 3
        elif self.counter < 500:
            
            V_x = ( 0 - self.initial_x  ) / 10
            V_y = ( -1.5 - self.initial_y  ) / 10

            self.goal_x = V_x * ( self.counter - 300) * 0.05 + self.initial_x
            self.goal_y = V_y * ( self.counter - 300) * 0.05 + self.initial_y
            self.goal_z = 1.0
            self.desired_yaw = 0

            
            self.path_controller()

            if self.counter == 500 - 1:
                self.init_state = True
            phase = 4
        elif self.counter == 500:
            
            print("Land")
            self.land()
            self.phase_timer.cancel()
            phase = 5

        self.counter += 1
        if hasattr(self,"goal_x") and phase != 5:
            print(f" Goal_x = {self.goal_x} - Goal_y = {self.goal_y} - Goal_yaw = {self.desired_yaw}")

    def quaternion_to_euler(self , x, y, z, w):

        rotation = Rotation.from_quat([x, y, z, w])
        roll, pitch, yaw = rotation.as_euler('xyz', degrees=False)  # 'xyz' convention
        return roll, pitch, yaw

    def ot_camera_feedback(self, pose_msg):
        for body in pose_msg.rigid_bodies:
            if body.id == self.robot_id:
                pose = body.pose_stamped.pose
                self.robot_x = pose.position.x
                self.robot_y = pose.position.y
                self.robot_z = pose.position.z

                x_q = pose.orientation.x
                y_q = pose.orientation.y
                z_q = pose.orientation.z
                w_q = pose.orientation.w

                _, _, self.robot_yaw = self.quaternion_to_euler(x_q, y_q, z_q, w_q)

                if self.init_state == True:
                    self.initial_x = self.robot_x
                    self.initial_y = self.robot_y
                    self.initial_yaw = self.robot_yaw
                    self.init_state = False


    def take_off (self):
        print("Take off!")
        if self.take_off_cmd == False:
            cmd_takeoff = Empty()
            self.pulisher_take_off.publish(cmd_takeoff)
            self.take_off_cmd = True
        else:
            return None

    def land(self):
        cmd_stop = Twist()  # Zero all velocities
        self.pulisher_cmd_vel.publish(cmd_stop)
        self.get_logger().info("Published zero velocity before landing.")

        time.sleep(0.5)  # Let it stabilize

        cmd_land = Empty()

        for i in range(5):  # Retry publishing
            if self.pulisher_land.get_subscription_count() > 0:
                self.pulisher_land.publish(cmd_land)
                self.get_logger().info(f"Landing command attempt {i+1}")
            else:
                self.get_logger().warn("No subscriber on /bebop/land")
            time.sleep(0.1)

    def global_to_local(self , x, y, x0, y0, theta_robot):
        
        theta_total = theta_robot - math.pi / 2  # add 90 degrees

        dx = x - x0
        dy = y - y0

        x_prime =  math.cos(theta_total) * dx + math.sin(theta_total) * dy
        y_prime = -math.sin(theta_total) * dx + math.cos(theta_total) * dy

        return x_prime, y_prime

    def path_controller (self):
        # Time measurement

        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time if hasattr(self, 'last_time') else 0.01
        self.last_time = current_time
        
        goal_x_local  , goal_y_local  = self.global_to_local(self.goal_x , self.goal_y , self.robot_x, self.robot_y, self.robot_yaw)

        yaw_error = self.desired_yaw - self.robot_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        derivative_yaw = (yaw_error - self.prev_error_yaw) / dt
        self.prev_error_yaw = yaw_error

        self.integral_yaw += yaw_error * dt

        error_x = goal_x_local - 0
        error_y = goal_y_local - 0
        error_z = self.goal_z  - self.robot_z

        self.integral_x += error_x * dt
        self.integral_y += error_y * dt
        self.integral_z += error_z * dt

        derivative_x = (error_x - self.prev_error_x) / dt
        derivative_y = (error_y - self.prev_error_y) / dt
        derivative_z = (error_z - self.prev_error_z) / dt        

        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_error_z = error_z      

        distance_to_goal = math.sqrt(error_x**2 + error_y**2)   

            
        cmd = Twist()
        
        cmd.linear.x = self.P_x * error_x + self.I_x * self.integral_x + self.D_x * derivative_x
        cmd.linear.y = self.P_y * error_y + self.I_y * self.integral_y + self.D_y * derivative_y
        cmd.linear.z = self.P_z * error_z + self.I_z * self.integral_z + self.D_z * derivative_z
        
        cmd.angular.z = self.P_yaw * yaw_error + self.I_yaw * self.integral_yaw + self.D_yaw * derivative_yaw

        self.pulisher_cmd_vel.publish(cmd)

        #print(f"Counter: {self.counter} - Robot X: {self.robot_x} - Robot Y: {self.robot_y} - Robot Z: {self.robot_z}")
        #print(f"Counter = {self.counter} - Psi = {self.robot_yaw}")

        # print(f"error_x: {error_x} - error_y: {error_y} - error_z: {error_z} - yaw: {self.robot_yaw}")
        # print(f"Goal_x = {goal_x_local} - Goal_y = {goal_y_local}")
        #print("-----------------------------------------------------------------------")
               
    # def path_controller_simple (self):
    #     # Time measurement
    #     current_time = self.get_clock().now().nanoseconds / 1e9
    #     dt = current_time - self.last_time if hasattr(self, 'last_time') else 0.01
    #     self.last_time = current_time
        
    #     goal_x_local  , goal_y_local  = self.global_to_local(self.goal_x , self.goal_y , self.robot_x, self.robot_y, self.robot_yaw)

    #     if self.counter <100:
    #         desired_yaw = 0
    #     else:
    #         desired_yaw = 0
    #         # self.goal_z = 0.5

    #     yaw_error = desired_yaw - self.robot_yaw
    #     yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

    #     derivative_yaw = (yaw_error - self.prev_error_yaw) / dt
    #     self.prev_error_yaw = yaw_error

    #     self.integral_yaw += yaw_error * dt

    #     error_x = goal_x_local - 0
    #     error_y = goal_y_local - 0
    #     error_z = self.goal_z  - self.robot_z

    #     self.integral_x += error_x * dt
    #     self.integral_y += error_y * dt
    #     self.integral_z += error_z * dt

    #     derivative_x = (error_x - self.prev_error_x) / dt
    #     derivative_y = (error_y - self.prev_error_y) / dt
    #     derivative_z = (error_z - self.prev_error_z) / dt        

    #     self.prev_error_x = error_x
    #     self.prev_error_y = error_y
    #     self.prev_error_z = error_z      

    #     distance_to_goal = math.sqrt(error_x**2 + error_y**2)   
        
    #     print(f"error_x: {error_x} - error_y: {error_y} - error_z: {error_z} - yaw: {self.robot_yaw}")
    #     print(f"Goal_x = {goal_x_local} - Goal_y = {goal_y_local}")
    #     #print("-----------------------------------------------------------------------")
    #     self.counter += 1
        

        # if self.counter == 200:
        #     print("Land")
        #     self.timer.cancel()
        #     self.land()
            
        # elif self.counter<200:
        #     cmd = Twist()
            
        #     cmd.linear.x = self.P_x * error_x + self.I_x * self.integral_x + self.D_x * derivative_x
        #     cmd.linear.y = self.P_y * error_y + self.I_y * self.integral_y + self.D_y * derivative_y
        #     cmd.linear.z = self.P_z * error_z + self.I_z * self.integral_z + self.D_z * derivative_z
            
        #     cmd.angular.z = self.P_yaw * yaw_error + self.I_yaw * self.integral_yaw + self.D_yaw * derivative_yaw

        #     self.pulisher_cmd_vel.publish(cmd)
        #     #print(f"Counter: {self.counter} - Robot X: {cmd.linear.x} - Robot Y: {cmd.linear.y} - Robot Z: {cmd.linear.z}")
        #     print(f"Counter = {self.counter} - Psi = {cmd.angular.z}")
        

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received! Initiating landing...")
        subprocess.run(['ros2','topic','pub','--once','/bebop/cmd_vel','geometry_msgs/msg/Twist'])
        subprocess.run(['ros2','topic','pub','--once','/bebop/land','std_msgs/msg/Empty'])
    finally:    
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()