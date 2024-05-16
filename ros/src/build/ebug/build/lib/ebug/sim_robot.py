# sim_robot.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class SimRobot(Node):

    def __init__(self):
        super().__init__('sim_robot')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f"Received cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}")
        status_msg = String()
        status_msg.data = f"Simulated movement: linear={msg.linear.x}, angular={msg.angular.z}"
        self.publisher_.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
