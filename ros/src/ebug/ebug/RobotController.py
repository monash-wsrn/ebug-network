"""
Handles the PID controller of the robot and i2c communication with Romi board
"""
import os
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from ebug.util.PololuHardwareInterface import PololuHardwareInterface
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
        import time

        self.max_retry_i2c = 10                                         # TODO make into parameter instead
        self.bridge = PololuHardwareInterface(self.max_retry_i2c)       
        time.sleep(0.5)

        self.frequency = float(os.getenv('I2C_FREQUENCY', "100.0"))     # TODO make into parameter instead
        self.timer = self.create_timer(1.0 / self.frequency, self.odom_pose_update)

        self.odom_pub = self.create_publisher(Odometry, 'odometry', 10)
        self.control_sub =  self.create_subscription(ControlCommand, 'control', self.control_callback, 10)
        
        self.robot_id = os.getenv('ROBOT_ID', "default")
        self.start = True

        self.odom_x, self.odom_y, self.odom_th = 0.0, 0.0, 0.0

        self.motors(0, 0)
        self.lights(0, 0, 0)
        self.pencode_l, self.pencode_r = self.read_encoders_gyro()
        
    
    

    #### I2C Interaction ####
    
    def alive(self):
        on_error = lambda : self.get_logger().info("I/O error writing heartbeat")
        return self.bridge.write_alive(on_error)
                
    def motors(self, left, right):        
        on_error = lambda : self.get_logger().info("I/O error writing to motors")
        return self.bridge.write_motors(int(left), int(right), on_error)
    
    def lights(self, r, g, b):
        on_error = lambda : self.get_logger().info("I/O error writing to LED ring")
        return self.bridge.write_led_ring(int(r), int(g), int(b), on_error)
    
    def encoders(self):
        on_error = lambda : self.get_logger().info("I/O error reading from encoders")
        return self.bridge.read_encoders(on_error)

    def gyroscope(self):
        on_error = lambda : self.get_logger().info("I/O error reading from encoders")
        return self.bridge.read_gyroscope(on_error)


    
    #### Differential Drive Odometry ####

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
        self.alive()

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

        wl = float(sencode_l) * ENC_CONST
        wr = float(sencode_r) * ENC_CONST
        
        odom_v = (wl + wr) * WHEEL_RAD / 2.0
        odom_w = (wr - wl) * WHEEL_RAD / BASELINE    # modified to adhre to REP103

        self.odom_th = self.odom_th + odom_w                            # TODO, this lesser than reality
        self.odom_x  = self.odom_x  + odom_v * math.cos(self.odom_th)
        self.odom_y  = self.odom_y  + odom_v * math.sin(self.odom_th)

        t = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quat(0.0, 0.0, self.odom_th)

        odom = Odometry()
        odom.header.frame_id = f'{self.robot_id}_odom'
        odom.header.stamp = t
        odom.child_frame_id = f'{self.robot_id}'

        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = float(qx)
        odom.pose.pose.orientation.y = float(qy)
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)
        odom.pose.covariance = mat6diag(1e-1)   # The greater the change in position, the greater the covariance

        odom.twist.twist.linear.x = float(odom_v) / dt
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = float(odom_w) / dt
        odom.twist.covariance = mat6diag(1e-1)

        self.odom_pub.publish(odom) 
        

        
    #### Differential Drive Control ####
    def drive(self,v_desired,w_desired):
        # https://automaticaddison.com/calculating-wheel-velocities-for-a-differential-drive-robot/
        factor = float(w_desired * BASELINE) / 2.0
        wl_desired = float(v_desired - factor) / WHEEL_RAD
        wr_desired = float(v_desired + factor) / WHEEL_RAD
        
        # https://pololu.github.io/romi-32u4-arduino-library/class_romi32_u4_motors.html#a1c19beaeeb5a86a9d1ab7e054c825c13
        return (wl_desired * 7, wr_desired * 7)   # -300 is full reverse, +300 is full forward

    def control_callback(self, msg:ControlCommand):
        lduty, rduty = self.drive(msg.control.linear.x, msg.control.angular.z)

        self.motors(lduty, rduty)
        self.lights(msg.color.x, msg.color.y, msg.color.z)




def quat(roll, pitch, yaw):
    qx = math.sin(roll/2.0) * math.cos(pitch/2.0) * math.cos(yaw/2.0) - math.cos(roll/2.0) * math.sin(pitch/2.0) * math.sin(yaw/2.0)
    qy = math.cos(roll/2.0) * math.sin(pitch/2.0) * math.cos(yaw/2.0) + math.sin(roll/2.0) * math.cos(pitch/2.0) * math.sin(yaw/2.0)
    qz = math.cos(roll/2.0) * math.cos(pitch/2.0) * math.sin(yaw/2.0) - math.sin(roll/2.0) * math.sin(pitch/2.0) * math.cos(yaw/2.0)
    qw = math.cos(roll/2.0) * math.cos(pitch/2.0) * math.cos(yaw/2.0) + math.sin(roll/2.0) * math.sin(pitch/2.0) * math.sin(yaw/2.0)
    return qx, qy, qz, qw


def mat6diag(v):
    return [(float(v) if i % 7 == 0 else 0.0) for i in range(36)]


def main():
    rclpy.init()
    node = RobotController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        return
    finally:
        node.motors(0, 0)
        node.lights(0, 0, 0)
        node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()