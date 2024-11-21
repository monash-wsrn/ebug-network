import os
import math
import rclpy
import time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from ebug.util.PololuHardwareInterface import PololuHardwareInterface

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.robot_id = os.getenv('ROBOT_ID', "default")
        
        # Initialize hardware interface
        self.bridge = PololuHardwareInterface(retry_max=5)
        if not self.bridge.is_calibrated:
            self.get_logger().error("IMU calibration failed!")
            return
        self.get_logger().info("IMU calibrated successfully")
        self.get_logger().info(f"Gyro bias: x={self.bridge.gyro_bias['x']:.3f}, "
                          f"y={self.bridge.gyro_bias['y']:.3f}, "
                          f"z={self.bridge.gyro_bias['z']:.3f}")

        self.bridge.reset_odometry()
        self.get_logger().info("Reset odometry at startup")

        # Initialize state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.last_cmd = Twist()

        # IMU integration variables
        self.imu_orientation = 0.0
        self.last_imu_time = None
        # self.imu_orientation_covariance = [0.01, 0.0, 0.0,
        #                                  0.0, 0.01, 0.0,
        #                                  0.0, 0.0, 0.01]

        # Set up ROS publishers/subscribers
        cmd_vel_topic = f'/{self.robot_id}/cmd_vel'
        self.cmd_vel_sub = self.create_subscription(Twist, cmd_vel_topic, 
                                                  self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 
                                            f'/{self.robot_id}/odom', 10)
        self.imu_pub = self.create_publisher(Imu, 
                                           f'/{self.robot_id}/imu', 10)
        
        # Create update timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(f"Subscribed to {cmd_vel_topic}")

        # Initialize robot
        self.reset_velocities()

    def reset_velocities(self):
        """Send zero velocities to ensure the robot starts from a stop."""
        self.get_logger().info("Resetting velocities to zero")
        self.bridge.write_velocity(0.0, 0.0)

    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands."""
        self.last_cmd = msg
        self.current_linear_velocity = msg.linear.x
        self.current_angular_velocity = msg.angular.z
        self.get_logger().info(f"Received Twist - Linear: {self.current_linear_velocity} m/s, "
                              f"Angular: {self.current_angular_velocity} rad/s")

    def send_command(self):
        """Send velocity commands to hardware."""
        self.bridge.write_velocity(self.last_cmd.linear.x, self.last_cmd.angular.z)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def create_imu_message(self, wx, wy, wz, current_time):
        """Create and populate IMU message."""
        if self.last_imu_time is not None:
            dt = (current_time - self.last_imu_time).nanoseconds / 1e9
            self.imu_orientation += wz * dt
            self.imu_orientation = math.atan2(math.sin(self.imu_orientation), 
                                            math.cos(self.imu_orientation))
        self.last_imu_time = current_time

        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = self.robot_id
        q = self.quaternion_from_euler(0, 0, self.imu_orientation)
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        # imu_msg.orientation_covariance = self.imu_orientation_covariance
        imu_msg.angular_velocity.x = wx
        imu_msg.angular_velocity.y = wy
        imu_msg.angular_velocity.z = wz
        
        return imu_msg

    def create_odom_message(self, linear_vel, angular_vel, current_time):
        """Create and populate odometry message."""
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = f"{self.robot_id}/odom"
        odom_msg.child_frame_id = self.robot_id

        # Set position and orientation
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        q = self.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Set velocities
        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = angular_vel

        # Set covariance
        # covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
        #              0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
        #              0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
        #              0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
        #              0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
        #              0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        # odom_msg.pose.covariance = covariance
        # odom_msg.twist.covariance = covariance
        
        return odom_msg

    def timer_callback(self):
        """Main update loop."""
        self.send_command()
        odom_data = self.bridge.read_odometry()
        gyro_data = self.bridge.read_gyroscope()

        if odom_data and gyro_data:
            # Unpack sensor data
            x_delta, y_delta, theta_delta, linear_vel, angular_vel = odom_data
            wx, wy, wz = gyro_data
            current_time = self.get_clock().now()

            # Update state
            self.x = x_delta
            self.y = y_delta
            self.theta = theta_delta

            # Create and publish messages
            imu_msg = self.create_imu_message(wx, wy, wz, current_time)
            odom_msg = self.create_odom_message(linear_vel, angular_vel, current_time)
            
            self.imu_pub.publish(imu_msg)
            self.odom_pub.publish(odom_msg)
        else:
            self.get_logger().warn("Failed to read odometry or gyroscope data")

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.reset_velocities()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()