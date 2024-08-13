import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from ebug.util.PololuHardwareInterface import PololuHardwareInterface

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        self.robot_id = os.getenv('ROBOT_ID', "default")
        
        # Initialize the I2C bridge
        self.bridge = PololuHardwareInterface(retry_max=5, logger=self.get_logger())

        # Subscribe and Publish topics
        cmd_vel_topic = f'/{self.robot_id}/cmd_vel'
        self.cmd_vel_sub = self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, f'/{self.robot_id}/odom', 10)

        # Create a timer that calls timer_callback every 0.1 seconds
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f"Subscribed to {cmd_vel_topic}")

        # Reset velocities to zero at startup
        self.reset_velocities()

    def reset_velocities(self):
        """Send zero velocities to ensure the robot starts from a stop."""
        self.get_logger().info("Resetting velocities to zero")
        self.bridge.write_velocity(0.0, 0.0)

    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Log the received velocities
        self.get_logger().info(f"Received Twist - Linear: {linear_velocity} m/s, Angular: {angular_velocity} rad/s")

        # Send velocities to Arduino
        self.bridge.write_velocity(linear_velocity, angular_velocity)

    def timer_callback(self):
        # Read odometry data from the Arduino
        odom_data = self.bridge.read_odometry()
        if odom_data:
            x, y, theta = odom_data
            self.get_logger().info(f"Odometry data - x: {x}, y: {y}, theta: {theta}")
            
            # Create an Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"

            # Set the position
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = 0.0

            # Set the orientation
            q = quaternion_from_euler(0, 0, theta)
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]

            # Publish the odometry data
            self.odom_pub.publish(odom_msg)
        else:
            self.get_logger().warn("Failed to read odometry data")

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