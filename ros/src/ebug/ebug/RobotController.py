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

BASELINE = 0.1320                                           # Distance between wheels in meters
WHEEL_RAD = 0.0350 * MULTIPLIER                             # Wheel radius in meters
GEAR_RATIO = 3952.0 / 33.0                                  # Gear Ratio X:1
ENC_CPR = 12.0                                              # Encoders Counts-per-revolution
ENC_CONST = (2.0 * math.pi) / (ENC_CPR * GEAR_RATIO)        # Encoder constant
                                                            # https://pololu.github.io/romi-32u4-arduino-library/class_romi32_u4_encoders.html

class RobotController(Node):

    def __init__(self):
        super().__init__(self.__class__.__name__)
        import time

        self.max_retry_i2c = int(os.getenv('I2C_RETRIES', "256"))       # TODO make into parameter instead
        self.robot_id = os.getenv('ROBOT_ID', "default")                # TODO make into parameter instead
        self.frequency = float(os.getenv('I2C_FREQUENCY', "50.0"))      # TODO make into parameter instead
        
        
        self.bridge = PololuHardwareInterface(self.max_retry_i2c)       
        time.sleep(0.5)
        
        self.timer = self.create_timer(1.0 / self.frequency, self.odom_pose_update)

        self.odom_pub = self.create_publisher(Odometry, 'odometry', 10)
        self.control_sub =  self.create_subscription(ControlCommand, 'control', self.control_callback, 10)
        
        self.odom = (0.0, 0.0, 0.0) # (x, y, yaw)

        self.motors(0, 0)
        self.lights(0, 0, 0)
        
        self.timestamp = self.get_clock().now().nanoseconds
        self.lencoder, self.rencoder = self.encoders()
        
    
    

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
        
        result  = None
        while result is None:
            result = self.bridge.read_encoders(on_error)
        return result

    def gyroscope(self):
        on_error = lambda : self.get_logger().info("I/O error reading from encoders")

        result  = None
        while result is None:
            result = self.bridge.read_gyroscope(on_error)
        return result


    
    #### Differential Drive Odometry ####

    def delta_time(self):
        now = self.get_clock().now().nanoseconds
        delta = float(now - self.timestamp) / 1_000_000_000.0
        self.timestamp = now
        return delta
        
    
    # Kinematic motion model
    def odom_pose_update(self):
        self.alive()

        dt = self.delta_time()
        encl, encr = self.encoders()
        
        dl = float(encl - self.lencoder) * ENC_CONST * WHEEL_RAD        # Distance travelled by left wheel
        dr = float(encr - self.rencoder) * ENC_CONST * WHEEL_RAD        # Distance travelled by right wheel
        
        self.lencoder = encl
        self.rencoder = encr
        
        # https://robotics.stackexchange.com/a/1679
        # Update odometry
        x, y, yaw = self.odom
        dw = 0.0
        if (abs(dl - dr) < 1e-6):
            nx = x + dl * math.cos(yaw)
            ny = y + dr * math.sin(yaw)
            nyaw = yaw
            
            self.odom = (nx, ny, nyaw)
        else:
            R = (BASELINE * (dl + dr)) / (2.0 * (dr - dl))
            dw = (dr - dl) / BASELINE
            
            nx = x + R * math.sin(dw + yaw) - R * math.sin(yaw)
            ny = y - R * math.cos(dw + yaw) + R * math.cos(yaw)
            nyaw = math.remainder(yaw + dw, math.tau)           # Bound angle to [-pi, pi] -> https://stackoverflow.com/a/61485110
            
            self.odom = (nx, ny, nyaw)
        
        # Send out latest odometry
        x, y, yaw = self.odom
        qx, qy, qz, qw = quat(0.0, 0.0, yaw)
        t = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.frame_id = f'{self.robot_id}_odom'
        odom.header.stamp = t
        odom.child_frame_id = f'{self.robot_id}'

        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = float(qx)
        odom.pose.pose.orientation.y = float(qy)
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)
        odom.pose.covariance = mat6diag(1e-2)

        vx = (dl - dr) / 2.0 / dt
        vyaw = dw / dt
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = float(vyaw)
        odom.twist.covariance = mat6diag(1e-1)

        self.odom_pub.publish(odom) 
        

        
    #### Differential Drive Control ####
    def drive(self, v_desired, w_desired):
        # https://automaticaddison.com/calculating-wheel-velocities-for-a-differential-drive-robot/
        rotation = float(w_desired * BASELINE) / 2.0
        
        lenc_desired = float(v_desired - rotation) / WHEEL_RAD / ENC_CONST  # Calculate desired encoder counts per second
        renc_desired = float(v_desired + rotation) / WHEEL_RAD / ENC_CONST  # Calculate desired encoder counts per second
        return (lenc_desired, renc_desired)

    def control_callback(self, msg:ControlCommand):
        lenc_desired, renc_desired = self.drive(msg.control.linear.x, msg.control.angular.z)
        
        self.alive()
        self.motors(lenc_desired, renc_desired)
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