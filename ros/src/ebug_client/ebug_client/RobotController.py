"""
Handles the PID controller of the robot and i2c communication with Romi board
"""

from ebug_client.util.AStar import AStar
import rclpy
from rclpy.node import Node
import numpy as np
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation as R

# PATH = [(0.5, 0.0), (0.0, 0.5), (-0.5, 0.0), (0.0, -0.5), (0.5, 0.0), (1.0, 0.0)]
# PATH = [(0.5, 0.0), (0.5, 0.5), (-0.5, 0.5), (-0.5, -0.5), (0.5, -0.5), (0.5, 0.0), (1.0, 0.0)]

ENCODER=12*3952/33 # constant for encoder (readings per revolution)


class RobotController(Node):

    def __init__(self):
        super().__init__(self.__class__.__name__)

        self.a_star = AStar()

        self.timer = self.create_timer(0.1, self.odom_pose_update)

        self.odom_pub=self.create_publisher(Odometry, '/odometry', 10)

        # self.imu_pub = self.create_publisher(Imu, '/imu', 10)

        self.cmd_vel_sub =  self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.start = 1

        self.r = 0.035
        self.l = 0.14

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

        # Veclocity motion model
    def base_velocity(self,wl,wr):

        v = (wl*self.r + wr*self.r)/2.0
        
        w = (wr*self.r - wl*self.r)/self.l #modified to adhre to REP103
        
        return v, w

    def overflow_corr(self, enc, prev_enc):

        val = enc - prev_enc
        if val > 32768:
            val = val - 32768
        
        elif val < -32768:

            val = val + 32768

        return val
    
    def read_encoders_gyro(self):

        while True:

            try:
                prev_enc_l, prev_enc_r = self.a_star.read_encoders()
                self.wx, self.wy, self.wz = self.a_star.read_gyroscope() 
                return prev_enc_l, prev_enc_r
            except:
                continue
    
    def motors(self,left, right):

        while True:
            try:
                self.a_star.motors(int(left), int(right))
                return
            except:

                self.get_logger().info("I/O error")

                continue


    
    # Kinematic motion model
    def odom_pose_update(self):

        if self.start: # first value

            self.start = 0
            self.prev_t = time.time()
            self.prev_enc_l, self.prev_enc_r = self.read_encoders_gyro()

        else:
            current_t = time.time()
            dt = current_t-self.prev_t

            self.prev_t = current_t
            encoder_l, encoder_r = self.read_encoders_gyro()

            self.wl = min(max(2*np.pi*(self.overflow_corr(encoder_l, self.prev_enc_l))/dt/ENCODER, -12),12)
            self.wr = min(max(2*np.pi*(self.overflow_corr(encoder_r, self.prev_enc_r))/dt/ENCODER, -12),12)

            self.prev_enc_l=encoder_l
            self.prev_enc_r=encoder_r
            
            self.odom_v, self.odom_w = self.base_velocity(self.wl,self.wr)
            self.odom_th = self.odom_th + self.odom_w*dt
            self.odom_x = self.odom_x + dt*self.odom_v*np.cos(self.odom_th)
            self.odom_y = self.odom_y + dt*self.odom_v*np.sin(self.odom_th)
            

            t = self.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.frame_id = 'robot/odom'
            odom.header.stamp = t
            odom.child_frame_id ='robot'
            q = R.from_euler('xyz',[.0, .0, self.odom_th]).as_quat().astype('float')
            odom.pose.pose.position.x = self.odom_x
            odom.pose.pose.position.y = self.odom_y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = float(q[0])
            odom.pose.pose.orientation.y = float(q[1])
            odom.pose.pose.orientation.z = float(q[2])
            odom.pose.pose.orientation.w = float(q[3])
            odom.pose.covariance = np.diag([1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3]).ravel()
            odom.twist.twist.linear.x = float(self.odom_v)
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.linear.z = 0.0
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = float(self.odom_w)
            odom.twist.covariance = np.diag([1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3]).ravel()
            self.odom_pub.publish(odom) 


            # imu = Imu()
            # imu.header.frame_id = 'robot/odom'
            # imu.header.stamp = t

            # imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = 0.0,0.0,0.0,0.0
            # imu.orientation_covariance = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]

            # imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = float(self.wx), float(self.wy), float(self.wz)
            # imu.angular_velocity_covariance = [1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1]

            # imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = 0.0,0.0,0.0
            # imu.linear_acceleration_covariance = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]

            # self.imu_pub.publish(imu)


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

        return np.sqrt((goal_x-x)**2 + (goal_y-y)**2)

    def cmd_vel_callback(self, msg:Twist):

        self.desired_v = msg.linear.x
        self.desired_w = msg.angular.z

        duty_cycle_l,duty_cycle_r = self.drive(self.desired_v, self.desired_w, self.wl, self.wr)
        self.motors(duty_cycle_l, duty_cycle_r)



def main():
    rclpy.init()
    node = RobotController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        return
    finally:
        node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()