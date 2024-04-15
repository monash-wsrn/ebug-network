import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile

from rclpy.callback_groups import ReentrantCallbackGroup

from ebug_interfaces.srv import ComputeTarget
from ebug_interfaces.msg import RobotPose
from ebug_interfaces.msg import ControlCommand

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
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


    def compute_target(self, payload:ComputeTarget):
        exists = payload.robot_id in self.robot_poses               # String

        self.robot_poses[payload.robot_id] = payload.pose.pose.pose # Pull Pose from PoseWithCovarianceStamped
                                                                    # Contains (Point) 'position' and (Quaternion) 'orientation' 

        control = Twist()                                            # control
        control.linear.x = 0
        control.angular.z = 0

        result = ControlCommand()

        if exists:
            other_poses = []
            for key, value in self.robot_poses.items():
                if not key == payload.robot_id:
                    other_poses.append(value)               # Array of Pose (position & orientation)

            this_pose = self.robot_poses[payload.robot_id]  # Pose (position & orientation)

            # It could also just be left wheel power and right wheel power???
            linear_x, angular_z, led_colour = Boids.next(this_pose, other_poses)

            control.linear.x = linear_x  # poll time is the time it takes before the robot sends another service request
            control.angular.z = angular_z

            # TODO make color meaningful
            result.control = control
            result.color.x = led_colour[0]
            result.color.y = led_colour[1]
            result.color.z = led_colour[2]
        

        # Publish this movement globally, for use in simulation and visualisation
        robot_pose = RobotPose()
        robot_pose.robot_id = payload.robot_id
        robot_pose.pose = payload.pose
        self.global_poses.publish(robot_pose)

        return result




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