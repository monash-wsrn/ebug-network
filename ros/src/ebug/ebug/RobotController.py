import os
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from ebug.util.PololuHardwareInterface import PololuHardwareInterface

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        self.robot_id = os.getenv('ROBOT_ID', "default")
        
        # Initialize the I2C bridge
        self.bridge = PololuHardwareInterface(retry_max=5)

        # Reset odometry at startup
        self.bridge.reset_odometry()
        self.get_logger().info("Reset odometry at startup")

        # Subscribe and Publish topics
        cmd_vel_topic = f'/{self.robot_id}/cmd_vel'
        self.cmd_vel_sub = self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, f'/{self.robot_id}/odom', 10)
        self.imu_pub = self.create_publisher(Imu, f'/{self.robot_id}/imu', 10)  # IMU publisher

        # Create a timer that calls timer_callback every 0.1 seconds
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f"Subscribed to {cmd_vel_topic}")

        # Initialize variables to store the current velocities
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.last_cmd = Twist()

        # Initialize odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Reset velocities and odometry to zero at startup
        self.reset_velocities()
        self.reset_odometry()

    def reset_velocities(self):
        """Send zero velocities to ensure the robot starts from a stop."""
        self.get_logger().info("Resetting velocities to zero")
        self.bridge.write_velocity(0.0, 0.0)

    def reset_odometry(self):
        """Reset the odometry to start from zero."""
        self.get_logger().info("Resetting odometry to zero")
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def cmd_vel_callback(self, msg):
        # Store the current velocities
        self.last_cmd = msg
        self.current_linear_velocity = msg.linear.x
        self.current_angular_velocity = msg.angular.z

        # Log the received velocities
        self.get_logger().info(f"Received Twist - Linear: {self.current_linear_velocity} m/s, Angular: {self.current_angular_velocity} rad/s")

    
    def send_command(self):
        self.bridge.write_velocity(self.last_cmd.linear.x, self.last_cmd.angular.z)

    def timer_callback(self):
        # Read odometry data from the Arduino
        self.send_command()
        odom_data = self.bridge.read_odometry()
        gyro_data = self.bridge.read_gyroscope()

        if odom_data and gyro_data:
            x_delta, y_delta, theta_delta, linear_vel, angular_vel = odom_data
            wx, wy, wz = gyro_data

            # Update odometry based on the deltas from the Arduino
            self.x = x_delta
            self.y = y_delta
            self.theta = theta_delta

            

            # Create and publish the IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "base_link"
            imu_msg.angular_velocity.x = wx
            imu_msg.angular_velocity.y = wy
            imu_msg.angular_velocity.z = wz
            self.imu_pub.publish(imu_msg)

            # Create and publish the Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = f"{self.robot_id}/odom"
            odom_msg.child_frame_id = self.robot_id
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0.0
            q = self.quaternion_from_euler(0, 0, self.theta)
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]

            # Use the current velocities from cmd_vel for the twist
            odom_msg.twist.twist.linear.x = linear_vel
            odom_msg.twist.twist.linear.y = 0.0  # Assuming no lateral velocity for differential drive
            odom_msg.twist.twist.angular.z = angular_vel

            odom_msg.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
            odom_msg.twist.covariance = odom_msg.pose.covariance

            self.odom_pub.publish(odom_msg)

        else:
            self.get_logger().warn("Failed to read odometry or gyroscope data")

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll / 2.0) * math.cos(pitch / 2.0) * math.cos(yaw / 2.0) - math.cos(roll / 2.0) * math.sin(pitch / 2.0) * math.sin(yaw / 2.0)
        qy = math.cos(roll / 2.0) * math.sin(pitch / 2.0) * math.cos(yaw / 2.0) + math.sin(roll / 2.0) * math.cos(pitch / 2.0) * math.sin(yaw / 2.0)
        qz = math.cos(roll / 2.0) * math.cos(pitch / 2.0) * math.sin(yaw / 2.0) - math.sin(roll / 2.0) * math.sin(pitch / 2.0) * math.cos(yaw / 2.0)
        qw = math.cos(roll / 2.0) * math.cos(pitch / 2.0) * math.cos(yaw / 2.0) + math.sin(roll / 2.0) * math.sin(pitch / 2.0) * math.sin(yaw / 2.0)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.reset_velocities()  # Reset velocities before shutting down
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
