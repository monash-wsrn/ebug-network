"""
Handles the PID controller of the robot and i2c communication with Romi board
"""
import os
import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3

from ebug.util.AStar import AStar
from ebug_base.msg import ControlCommand

BASELINE = 0.142                                            # Distance between wheels in meters
WHEEL_RAD = 0.0345                                          # Wheel radius in meter
GEAR_RATIO = 3952.0 / 33.0                                  # Gear Ratio X:1
ENC_CPR = 12.0                                              # Encoders Counts-per-revolution
ENC_CONST = (2.0 * math.pi) / (ENC_CPR * GEAR_RATIO)        # Encoder constant
                                                            # https://pololu.github.io/romi-32u4-arduino-library/class_romi32_u4_encoders.html

class RobotController(Node):

    def __init__(self):
        super().__init__(self.__class__.__name__)

        self.a_star = AStar()

        self.timer = self.create_timer(0.02, self.odom_pose_update)

        self.odom_pub = self.create_publisher(Odometry, 'odometry', 10)

        self.cmd_vel_sub =  self.create_subscription(ControlCommand, 'cmd_vel', self.cmd_vel_callback, 10)
        
        self.robot_id = os.getenv('ROBOT_ID', "default")
        self.start = 1

        self.r = WHEEL_RAD
        self.l = BASELINE

        self.odom_x, self.odom_y, self.odom_th = 0.0, 0.0, 0.0
        self.odom_v, self.odom_w = 0.0,  0.0
        self.e_prev = 0.0

        self.Kp = 2
        self.Ki = 0.9
        self.Kd = 5
        self.exp_alpha = 1

        self.wl, self.wr = 0.0,  0.0

        self.path_idx = 0
        self.done = 0

        self.duty_cycle_l, self.duty_cycle_r = 0, 0
        self.I = 0
        self.e_prev = 0

        self.timestamp = 0

    # Veclocity motion model
    def base_velocity(self,wl,wr):

        v = (wl*self.r + wr*self.r) / 2.0
        
        w = (wr*self.r - wl*self.r) / self.l    # modified to adhre to REP103
        
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
    
    # Kinematic motion model
    def odom_pose_update(self):
        dt = self.delta_time()
        if dt < 1e-9:
            return

        encoder_l, encoder_r = self.read_encoders_gyro()
        
        self.wl = encoder_l * ENC_CONST
        self.wr = encoder_r * ENC_CONST
        
        self.odom_v, self.odom_w = self.base_velocity(self.wl, self.wr)
        self.odom_th = self.odom_th + self.odom_w
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
        odom.pose.covariance = mat6diag(1e-2)

        odom.twist.twist.linear.x = float(self.odom_v) / dt
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = float(self.odom_w) / dt
        odom.twist.covariance = mat6diag(1e-2)

        self.odom_pub.publish(odom) 


    def p_control(self,duty_cycle, w_desired,w_measured):
        

        e = min(max(w_desired-w_measured, -200), 200)
        
        P = self.Kp*e
        self.I = self.I + self.Ki*e
        D = self.Kd*(e - self.e_prev)

        self.e_prev = e

        duty_cycle = self.exp_alpha*min(max(P + self.I + D, -200),200) + (1-self.exp_alpha)*duty_cycle
        return duty_cycle
        
        
    def drive(self,v_desired,w_desired,wl,wr):
        

        wl_desired = (v_desired - self.l*w_desired/2)/self.r
        wr_desired = (v_desired + self.l*w_desired/2)/self.r
        
        self.duty_cycle_l = wl_desired * 7 #self.p_control(self.duty_cycle_l, wl_desired,wl)*1.005
        self.duty_cycle_r = wr_desired * 7 #self.p_control(self.duty_cycle_r, wr_desired,wr)
        return self.duty_cycle_l, self.duty_cycle_r

    def distance(self,goal_x, goal_y, x, y):

        return math.sqrt((goal_x-x)**2 + (goal_y-y)**2)

    def cmd_vel_callback(self, msg:ControlCommand):

        self.desired_v = msg.control.linear.x
        self.desired_w = msg.control.angular.z

        duty_cycle_l,duty_cycle_r = self.drive(self.desired_v, self.desired_w, self.wl, self.wr)
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