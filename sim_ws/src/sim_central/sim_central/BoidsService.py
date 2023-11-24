import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sim_interfaces.srv import ComputeTarget
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

class BoidsService(Node):
    def __init__(self):
        super().__init__(self.__class__.__name__)

        # TODO apply given ros name and not class name as default value??
        self.declare_parameter('service_name', self.__class__.__name__)
        self.service_name = self.get_parameter('service_name').get_parameter_value().string_value

        self.service = self.create_service(ComputeTarget, self.service_name, self.compute_target)

        # This will be populated with each call to the compute_target function, it's
        # a dictionary of poses, where the key is the robot_id and the value is the
        # most current pose of the respective robot, as is reported by the robot itself.
        self.robot_poses = {}


    def compute_target(self, payload:ComputeTarget):
        first_update = payload.robot_id in self.robot_poses
        self.robot_poses[payload.robot_id] = payload.pose
        result = Twist()
        
        if not first_update:
            # TODO on every update, after the first, execute Boids logic
            # use the robot_poses map to get the current pose for each robot_id 

            # The real robots expect the following response parameters to be set,
            # as best as I can tell, this refers to the velocities and not the 
            # absolute deltas (i.e., speed not distance)

            # It could also just be left wheel power and right wheel power???
            result.linear.x = 0
            result.angular.z = 0
            pass

        return result




## Boilerplate

def main():
    rclpy.init()
    node = BoidsService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        return

    rclpy.shutdown()

if __name__ == '__main__':
    main()