import rclpy
from rclpy.node import Node
from rclpy import time
import numpy as np

from ebug_interfaces.srv import ComputeTarget
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Pose, PoseWithCovariance
from tf_transformations import quaternion_from_euler

class PseudoMovementController(Node):
    def __init__(self):
        super().__init__(self.__class__.__name__)

        self.declare_parameter('service_name', 'ComputeTargetService')
        self.service_name = self.get_parameter('service_name').get_parameter_value().string_value
        
        self.declare_parameter('start_pos', [0, 0])
        self.start_pos = self.get_parameter('start_pos').get_parameter_value().integer_array_value
        
        self.declare_parameter('start_yaw', 0.0)
        self.start_yaw = self.get_parameter('start_yaw').get_parameter_value().double_value

        self.client = self.create_client(ComputeTarget, self.service_name)
        
        self.pose = Pose()
        self.yaw = self.start_yaw
        self.pose.position.x = self.start_pos[0]
        self.pose.position.y = self.start_pos[1]
        self.pose.position.z = 0
        self.pose.orientation = quaternion_from_euler(0, 0, self.yaw)
        self.tick_rate = 25.0
        self.timer = self.create_timer(1.0 / self.tick_rate, self.tick)
        self.timestamp = None
        
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        
        self.get_logger().info(f"Created SudoMovementController (ID: {self.get_namespace()}) using {self.service_name}")
    

    """
    Upon receiving a pose update from a robot, calculate target velocity and 
    publish to back to the robot. It will do this through the central control
    service that has visibility of all robots' pose
    """
    def tick(self):
        if not self.client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('Service unavailable, no action undertaken')
            return

        if not self.timestamp:
            self.timestamp = time.Time()
        
        delta = (time.Time().nanoseconds - self.timestamp.nanoseconds) / 1_000_000_000.0
        self.timestamp = time.Time()

        # TODO apply delta time change with linear and angular velocities
        self.yaw = self.yaw + self.twist.angular.z*delta
        self.pose.position.x = self.pose.orientation.x + delta*self.twist.linear.x*np.cos(self.yaw)
        self.pose.position.y = self.pose.orientation.y + delta*self.twist.linear.x*np.sin(self.yaw)

        self.pose.orientation = quaternion_from_euler(0, 0, self.yaw)
        
        request = ComputeTarget.Request()
        request.robot_id = self.get_namespace()
        request.pose = PoseWithCovarianceStamped()
        request.pose.pose = PoseWithCovariance()
        request.pose.pose.pose = self.pose

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result() # Returns a ControlCommand
        self.twist = response.control
        self.timestamp = self.get_clock().now().to_msg()


## Boilerplate

def main():
    rclpy.init()
    node = PseudoMovementController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        return
    finally:
        node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':  
    main()