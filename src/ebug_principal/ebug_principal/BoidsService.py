import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from ebug_interfaces.srv import ComputeTarget
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

        # This will be populated with each call to the compute_target function, it's
        # a dictionary of poses, where the key is the robot_id and the value is the
        # most current pose of the respective robot, as is reported by the robot itself.
        self.robot_poses = {}


    def compute_target(self, payload:ComputeTarget):
        first_update = payload.robot_id in self.robot_poses         # String

        self.robot_poses[payload.robot_id] = payload.pose.pose.pose # Pull Pose from PoseWithCovarianceStamped
                                                                    # Contains (Point) 'position' and (Quaternion) 'orientation' 

        result = Twist()                                            # Twist
        result.linear.x = 0
        result.angular.z = 0

        if not first_update:

            other_poses = []
            for key, value in self.robot_poses.items():
                if not key == payload.robot_id:
                    other_poses.append(value)               # Array of Pose (position & orientation)

            this_pose = self.robot_poses[payload.robot_id]  # Pose (position & orientation)

            # It could also just be left wheel power and right wheel power???
            linear_x, angular_z = Boids.next(this_pose, other_poses)

            result.linear.x = linear_x  # poll time is the time it takes before the robot sends another service request
            result.angular.z = angular_z

            # TODO maybe apply temporal component to return value so that robots don't overshoot ??
            # result.linear.z = temporal ???
        
        return result




## Boilerplate

def main():
    rclpy.init()
    node = BoidsService()

    ## TODO define executor affinity via environment variable
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)

    node.get_logger().info("Created MultiThreadedExecutor for BoidsService!")
    
    try:
        # rclpy.spin(node)
        executor.spin()
    except KeyboardInterrupt:
        return
    finally:
        node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()