
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from rclpy.callback_groups import ReentrantCallbackGroup

from ebug_interfaces.srv import ComputeTarget
from ebug_interfaces.msg import RobotPose, ControlCommand

from geometry_msgs.msg import PoseWithCovariance, Twist

import ebug_principal.BoidsFunction as Boids

class BoidsService(Node):
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
        
        # This will be populated with each call to the compute_target function, it's
        # a dictionary of poses, where the key is the robot_id and the value is the
        # most current pose of the respective robot, as is reported by the robot itself.
        self.robot_poses = {}


    def compute_target(self, request, response):
        exists = request.robot_id in self.robot_poses               # String

        self.robot_poses[request.robot_id] = request.pose.pose # Pull Pose from PoseWithCovarianceStamped
                                                                    # Contains (Point) 'position' and (Quaternion) 'orientation' 

        response.control.linear.x = 0.0
        response.control.angular.z = 0.0

        if exists:
            other_poses = []
            for key, value in self.robot_poses.items():
                if not key == request.robot_id:
                    other_poses.append(value)               # Array of Pose (position & orientation)

            this_pose = self.robot_poses[request.robot_id]  # Pose (position & orientation)

            # It could also just be left wheel power and right wheel power???
            linear_x, angular_z, led_colour = Boids.next(this_pose, other_poses)

            response.control.linear.x = float(linear_x)     # poll time is the time it takes before the robot sends another service request
            response.control.angular.z = float(angular_z)

            # TODO make color meaningful
            response.color.x = float(led_colour[0])
            response.color.y = float(led_colour[1])
            response.color.z = float(led_colour[2])
        else:
            self.get_logger().info(f'Registered new robot with id: {request.robot_id}.')


        # Publish this movement globally, for use in simulation and visualisation
        robot_pose = RobotPose()
        robot_pose.robot_id = request.robot_id
        robot_pose.pose = request.pose
        self.global_poses.publish(robot_pose)

        return response




## Boilerplate

def main():
    rclpy.init()
    node = BoidsService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        return
    finally:
        node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()