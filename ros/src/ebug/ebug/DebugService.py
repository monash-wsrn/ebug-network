
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from rclpy.callback_groups import ReentrantCallbackGroup

from ebug_base.srv import ComputeTarget
from ebug_base.msg import RobotPose

import os
FORWARD_SPEED = float(os.getenv('MAX_FORWARD_SPEED', "0.050000"))       # m /s 
ANGULAR_SPEED = float(os.getenv('MAX_ANGULAR_SPEED', "0.785398"))       # rads /s

class DebugService(Node):
    def __init__(self):
        super().__init__(self.__class__.__name__)

        # TODO apply given ros name and not class name as default value??
        self.declare_parameter('service_name', self.__class__.__name__)
        
        # self.service_name = self.get_parameter('service_name').get_parameter_value().string_value
        self.service_name = self.get_name()

        self.cb_group = ReentrantCallbackGroup()
        self.service = self.create_service(ComputeTarget, self.service_name, self.compute_target, callback_group=self.cb_group)

        qos_profile = QoSProfile(depth=10)
        self.global_poses = self.create_publisher(RobotPose, "global_poses", qos_profile)


    def compute_target(self, request, response):
        response.control.linear.x = FORWARD_SPEED
        response.control.angular.z = ANGULAR_SPEED

        response.color.x = 255
        response.color.x = 0
        response.color.x = 255

        self.get_logger().info(f'Handled request from robot with id: {request.robot_id}.')
        return response




## Boilerplate

def main():
    rclpy.init()
    node = DebugService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        return
    finally:
        node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()