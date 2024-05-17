"""
Handles the PID controller of the robot and i2c communication with Romi board
"""
import os
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3

from ebug.util.AStar import AStar
from ebug_base.msg import ControlCommand

# https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
MULTIPLIER = float(os.getenv('WHEEL_MULT', "1.0862"))
BASELINE = 0.138                                            # Distance between wheels in meters
WHEEL_RAD = 0.0351 * MULTIPLIER                             # Wheel radius in meters
GEAR_RATIO = 3952.0 / 33.0                                  # Gear Ratio X:1
ENC_CPR = 12.0                                              # Encoders Counts-per-revolution
ENC_CONST = (2.0 * math.pi) / (ENC_CPR * GEAR_RATIO)        # Encoder constant
                                                            # https://pololu.github.io/romi-32u4-arduino-library/class_romi32_u4_encoders.html

class RobotController(Node):

    def __init__(self):
        super().__init__(self.__class__.__name__)

        self.a_star = AStar()

        self.frequency = float(os.getenv('ODOM_FREQUENCY', "40.0"))
        self.timer = self.create_timer(1.0 / self.frequency, self.odom_pose_update)

        self.odom_pub = self.create_publisher(Odometry, 'odometry', 10)

        self.cmd_vel_sub =  self.create_subscription(ControlCommand, 'cmd_vel', self.cmd_vel_callback, 10)
        
        self.robot_id = os.getenv('ROBOT_ID', "default")
        self.start = True

        self.r = WHEEL_RAD
        self.l = BASELINE

        self.odom_x, self.odom_y, self.odom_th = 0.0, 0.0, 0.0
        self.odom_v, self.odom_w = 0.0,  0.0

        self.pencode_l, self.pencode_r = self.read_encoders_gyro()
        self.wl, self.wr = 0.0, 0.0
        self.timestamp = 0

        self.duty_cycle_l, self.duty_cycle_r = 0, 0
        
        
    # Veclocity motion model
    def base_velocity(self,wl,wr):

        v = (wl + wr) * self.r / 2.0
        w = (wr - wl) * self.r / self.l    # modified to adhre to REP103
        
        return v, w
    
    def read_encoders_gyro(self):

        while True:
            try:
                prev_enc_l, prev_enc_r = self.a_star.read_encoders()
                self.wx, self.wy, self.wz = self.a_star.read_gyroscope() 
                return prev_enc_l, prev_enc_r
            except:
                continue
    

    def motors(self,left, right, led):
        self.try_i2c(lambda : self.a_star.motors(int(left), int(right)), "I/O error moving motors")
        self.try_i2c(lambda : self.a_star.led_ring(int(led.x), int(led.y), int(led.z)), "I/O error setting LEDs")
    

    def try_i2c(self, i2c_func, msg):
        for _ in range(10):
            try:
                i2c_func()
                return
            except:
                continue
        self.get_logger().info(msg)


    def delta_time(self):
        def ns():
            return self.get_clock().now().nanoseconds

        if self.timestamp == 0:
            self.timestamp = ns()
        
        delta = float(ns() - self.timestamp) / 1_000_000_000.0
        self.timestamp = ns()

        return delta
    
    def encoder_congruence(self, encoder_l, encoder_r):
        ldiff = int(encoder_l) - int(self.pencode_l)
        rdiff = int(encoder_r) - int(self.pencode_r)

        if (ldiff > 32767):
            ldiff -= 65535
        elif (ldiff < -32768):
            ldiff += 65536

        if (rdiff > 32767):
            rdiff -= 65535
        elif (rdiff < -32768):
            rdiff += 65536

        self.pencode_l = int(encoder_l)
        self.pencode_r = int(encoder_r)
        return ldiff, rdiff

    
    # Kinematic motion model
    def odom_pose_update(self):
        dt = self.delta_time()
        encoder_l, encoder_r = self.read_encoders_gyro()

        if (self.start):
            self.start = False
            self.pencode_l, self.pencode_r = int(encoder_l), int(encoder_r)
            return
        
        sencode_l, sencode_r = self.encoder_congruence(encoder_l, encoder_r)

        # Filter out spikes, allow no more than one wheel rotation per update 
        if abs(sencode_l) > 1440 or abs(sencode_r) > 1440:
            return

        self.wl = float(sencode_l) * ENC_CONST
        self.wr = float(sencode_r) * ENC_CONST
        
        self.odom_v, self.odom_w = self.base_velocity(self.wl, self.wr)
        self.odom_th = self.odom_th + self.odom_w # TODO, this lesser than reality
        self.odom_x = self.odom_x + self.odom_v * math.cos(self.odom_th)
        self.odom_y = self.odom_y + self.odom_v * math.sin(self.odom_th)

        t = self.get_clock().now().to_msg()
        q = quat(0.0, 0.0, self.odom_th)

        odom = Odometry()
        odom.header.frame_id = f'{self.robot_id}_odom'
        odom.header.stamp = t
        odom.child_frame_id = f'{self.robot_id}'

        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = float(q.x)
        odom.pose.pose.orientation.y = float(q.y)
        odom.pose.pose.orientation.z = float(q.z)
        odom.pose.pose.orientation.w = float(q.w)
        odom.pose.covariance = mat6diag(1e-3)

        odom.twist.twist.linear.x = float(self.odom_v) / dt
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = float(self.odom_w) / dt
        odom.twist.covariance = mat6diag(1e-3)

        self.odom_pub.publish(odom) 
        
    def drive(self,v_desired,w_desired):
        # https://automaticaddison.com/calculating-wheel-velocities-for-a-differential-drive-robot/
        factor = float(w_desired * self.l) / 2.0
        wl_desired = float(v_desired - factor) / self.r
        wr_desired = float(v_desired + factor) / self.r
        
        # https://pololu.github.io/romi-32u4-arduino-library/class_romi32_u4_motors.html#a1c19beaeeb5a86a9d1ab7e054c825c13
        self.duty_cycle_l = wl_desired * 7  # -300 is full reverse, +300 is full forward
        self.duty_cycle_r = wr_desired * 7  # -300 is full reverse, +300 is full forward

        return self.duty_cycle_l, self.duty_cycle_r

    def cmd_vel_callback(self, msg:ControlCommand):

        self.desired_v = msg.control.linear.x
        self.desired_w = msg.control.angular.z

        duty_cycle_l,duty_cycle_r = self.drive(self.desired_v, self.desired_w)
        self.motors(duty_cycle_l, duty_cycle_r, msg.color)


def quat(roll, pitch, yaw):
    q = Quaternion()
    q.x = math.sin(roll/2.0) * math.cos(pitch/2.0) * math.cos(yaw/2.0) - math.cos(roll/2.0) * math.sin(pitch/2.0) * math.sin(yaw/2.0)
    q.y = math.cos(roll/2.0) * math.sin(pitch/2.0) * math.cos(yaw/2.0) + math.sin(roll/2.0) * math.cos(pitch/2.0) * math.sin(yaw/2.0)
    q.z = math.cos(roll/2.0) * math.cos(pitch/2.0) * math.sin(yaw/2.0) - math.sin(roll/2.0) * math.sin(pitch/2.0) * math.cos(yaw/2.0)
    q.w = math.cos(roll/2.0) * math.cos(pitch/2.0) * math.cos(yaw/2.0) + math.sin(roll/2.0) * math.sin(pitch/2.0) * math.sin(yaw/2.0)
    return q

def mat6diag(v):
    return [(float(v) if i % 7 == 0 else 0.0) for i in range(36)]

def main():
    rclpy.init()
    node = RobotController()
    

    zero = Vector3()
    zero.x, zero.y, zero.z = 0.0, 0.0, 0.0
    
    node.motors(0, 0, zero)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        return
    finally:
        node.motors(0, 0, zero)
        node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()