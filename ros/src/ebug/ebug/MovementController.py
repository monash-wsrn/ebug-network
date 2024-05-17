import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from ebug_base.srv import ComputeTarget
from ebug_base.msg import ControlCommand

from nav_msgs.msg import Odometry

class MovementController(Node):
    def __init__(self):
        super().__init__(self.__class__.__name__)

        self.declare_parameter('service_name', 'ComputeTargetService')
        self.service_name = self.get_parameter('service_name').get_parameter_value().string_value

        
        self.declare_parameter('frequency', 30.0)
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value

        self.robot_id = os.getenv('ROBOT_ID', "default")

        # TODO shouldn't be the service module but rather the DTO and the service name
        self.client = self.create_client(ComputeTarget, f'/{self.service_name}')

        while not self.client.wait_for_service(timeout_sec=0.5):
            pass

        # Ideally we'd update our robots to interact directly with the BoidsService
        # as opposed to this roundabout way that allows the existing pub-sub
        # architecture to utilise a the service model
        qos_profile = QoSProfile(depth=1)
        self.pub_target = self.create_publisher(ControlCommand, "cmd_vel", qos_profile)
        
        self.sub_location = self.create_subscription(Odometry, "ekf_absolute", self.compute_target, qos_profile)
        self.get_logger().info(f"Created MovementController (ID: {self.get_namespace()}) using {self.service_name}")


    """
    Upon receiving a pose update from a robot, calculate target velocity and 
    publish to back to the robot. It will do this through the central control
    service that has visibility of all robots' pose
    """
    def compute_target(self, payload: Odometry):

        if not self.client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('Service unavailable, no action undertaken')
            return
        
        request = ComputeTarget.Request()
        request.robot_id = self.robot_id
        request.pose = payload.pose

        future = self.client.call_async(request)
        future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        result = future.result() # Returns a ControlCommand
        
        response = ControlCommand()
        response.control = result.control   # Twist
        response.color = result.color       # Vector3

        self.pub_target.publish(response)

def mat6diag(v):
    return [(float(v) if i % 7 == 0 else 0.0) for i in range(36)]

## Boilerplate

def main():
    rclpy.init()
    node = MovementController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        return
    finally:
        node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()