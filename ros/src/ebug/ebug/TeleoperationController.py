import os
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

from ebug.util.AStar import AStar

# Constants
MULTIPLIER = float(os.getenv('WHEEL_MULT', "1.0862"))
BASELINE = 0.142                                            # Distance between wheels in meters
WHEEL_RAD = 0.0351 * MULTIPLIER                             # Wheel radius in meters

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.a_star = AStar()
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        self.robot_id = os.getenv('ROBOT_ID', "default")
        
        self.r = WHEEL_RAD
        self.l = BASELINE

    def motors(self, left, right):
        self.try_i2c(lambda : self.a_star.motors(int(left), int(right)), "I/O error moving motors")
    
    def try_i2c(self, i2c_func, msg):
        for _ in range(10):
            try:
                i2c_func()
                return
            except:
                continue
        self.get_logger().info(msg)
        
    def drive(self, v_desired, w_desired):
        # https://automaticaddison.com/calculating-wheel-velocities-for-a-differential-drive-robot/
        factor = float(w_desired * self.l) / 2.0
        wl_desired = float(v_desired - factor) / self.r
        wr_desired = float(v_desired + factor) / self.r
        
        # Convert to duty cycle: -300 is full reverse, +300 is full forward
        duty_cycle_l = wl_desired * 7
        duty_cycle_r = wr_desired * 7

        return duty_cycle_l, duty_cycle_r

    def cmd_vel_callback(self, msg: Twist):
        v_desired = msg.linear.x
        w_desired = msg.angular.z

        duty_cycle_l, duty_cycle_r = self.drive(v_desired, w_desired)
        self.motors(duty_cycle_l, duty_cycle_r)


def main():
    rclpy.init()
    node = RobotController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        return
    finally:
        node.motors(0, 0)  # Stop motors
        node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
