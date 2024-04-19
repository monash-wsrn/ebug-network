import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from ebug_interfaces.srv import ComputeTarget
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseWithCovariance, Pose, Quaternion 

class PseudoMovementController(Node):
    def __init__(self):
        super().__init__(self.__class__.__name__)

        self.declare_parameter('service_name', 'ComputeTargetService')
        self.service_name = self.get_parameter('service_name').get_parameter_value().string_value
        
        self.declare_parameter('tick_rate', 25.0)
        self.tick_rate = self.get_parameter('tick_rate').get_parameter_value().double_value
        
        self.declare_parameter('start_pos', [0.0, 0.0])
        self.start_pos = self.get_parameter('start_pos').get_parameter_value().double_array_value
        
        self.declare_parameter('start_yaw', 0.0)
        self.start_yaw = self.get_parameter('start_yaw').get_parameter_value().double_value

        self.client = self.create_client(ComputeTarget, f'/{self.service_name}')

        while not self.client.wait_for_service(timeout_sec=0.5):
            pass
        
        self.pose = Pose()
        
        self.yaw = self.start_yaw
        self.pose.position.x = self.start_pos[0]
        self.pose.position.y = self.start_pos[1]
        self.pose.position.z = 0.0
        self.pose.orientation = self.quat()

        self.cb_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(1.0 / self.tick_rate, self.tick, self.cb_group, self.get_clock())
        self.timestamp = 0
        
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        
        self.get_logger().info(f"Created PseudoMovementController (ID: {self.get_namespace()}) using {self.service_name}")
    

    """
    Upon receiving a pose update from a robot, calculate target velocity and 
    publish to back to the robot. It will do this through the central control
    service that has visibility of all robots' pose
    """
    def tick(self):
        if not self.client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('Service unavailable, no action undertaken')
            return

        delta = self.delta_time()
        self.yaw                += delta * self.twist.angular.z
        self.pose.position.x    += delta * self.twist.linear.x * math.cos(self.yaw)
        self.pose.position.y    += delta * self.twist.linear.x * math.sin(self.yaw)
        self.pose.orientation = self.quat()
        
        request = ComputeTarget.Request()
        request.robot_id = self.get_namespace()
        request.pose = PoseWithCovarianceStamped()
        request.pose.pose = PoseWithCovariance()
        request.pose.pose.pose = self.pose

        future = self.client.call_async(request)
        future.add_done_callback(self.future_callback)

        self.timer.reset()
        
    def future_callback(self, future):
        self.twist = future.result().control
        

    def quat(self):
        q = Quaternion()
        roll, pitch, yaw = 0.0, 0.0, self.yaw

        q.x = math.sin(roll/2.0) * math.cos(pitch/2.0) * math.cos(yaw/2.0) - math.cos(roll/2.0) * math.sin(pitch/2.0) * math.sin(yaw/2.0)
        q.y = math.cos(roll/2.0) * math.sin(pitch/2.0) * math.cos(yaw/2.0) + math.sin(roll/2.0) * math.cos(pitch/2.0) * math.sin(yaw/2.0)
        q.z = math.cos(roll/2.0) * math.cos(pitch/2.0) * math.sin(yaw/2.0) - math.sin(roll/2.0) * math.sin(pitch/2.0) * math.cos(yaw/2.0)
        q.w = math.cos(roll/2.0) * math.cos(pitch/2.0) * math.cos(yaw/2.0) + math.sin(roll/2.0) * math.sin(pitch/2.0) * math.sin(yaw/2.0)
        return q

    def delta_time(self):
        def ns():
            return self.get_clock().now().nanoseconds

        if self.timestamp == 0:
            self.timestamp = ns()
        
        delta = float(ns() - self.timestamp) / 1_000_000_000.0
        self.timestamp = ns()

        return delta


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