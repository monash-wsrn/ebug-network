import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from ebug.util.PololuHardwareInterface import PololuHardwareInterface

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        self.robot_id = os.getenv('ROBOT_ID', "default")
        
        # Initialize the I2C bridge
        self.bridge = PololuHardwareInterface(retry_max=5)

        # Subscribe and Publish topic
        cmd_vel_topic = f'/{self.robot_id}/cmd_vel'
        self.cmd_vel_sub = self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Pose, f'/{self.robot_id}/odom', 10)
        
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
        x, y, theta = self.bridge.read_odometry()

        # Create a Pose message with the odometry data
        odom_msg = Pose()
        odom_msg.position.x = x
        odom_msg.position.y = y
        odom_msg.orientation.z = theta

        # Publish the odometry data
        self.odom_pub.publish(odom_msg)

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
