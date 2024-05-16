import os
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from ebug_base.msg import RobotPose

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
        self.global_pose_pub = self.create_publisher(RobotPose, '/global_poses', 10)

        self.robot_id = os.getenv('ROBOT_ID', "default")
        
        self.r = WHEEL_RAD
        self.l = BASELINE
        self.odom_x, self.odom_y, self.odom_th = 0.0, 0.0, 0.0

        # Timer to periodically update and publish robot's pose
        self.timer = self.create_timer(0.1, self.timer_callback)

    def motors(self, left, right):
        self.try_i2c(lambda : self.a_star.motors(int(left), int(right)), "I/O error moving motors")
        self.try_i2c(lambda : self.a_star.led_ring(int(0), int(0), int(255)), "I/O error setting LEDs to blue")
    
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
        if v_desired == 0 and w_desired != 0:
            # Pivoting on the spot
            wl_desired = -w_desired * self.l / self.r
            wr_desired = w_desired * self.l / self.r
        else:
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
        
        # Update robot's pose based on the velocity
        self.update_odometry(v_desired, w_desired)

    def timer_callback(self):
        # Publish the current pose at a fixed interval
        self.publish_robot_pose()

    def update_odometry(self, v, w):
        dt = 0.1  # Assuming a fixed time step for simplicity
        self.odom_th += w * dt
        self.odom_x += v * math.cos(self.odom_th) * dt
        self.odom_y += v * math.sin(self.odom_th) * dt

    def publish_robot_pose(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = f"{self.robot_id}_odom"
        odom_msg.child_frame_id = self.robot_id

        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.orientation.z = math.sin(self.odom_th / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.odom_th / 2.0)

        robot_pose = RobotPose()
        robot_pose.robot_id = self.robot_id
        robot_pose.pose = odom_msg.pose
        self.global_pose_pub.publish(robot_pose)


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
