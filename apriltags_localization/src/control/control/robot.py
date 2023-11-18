from .a_star import AStar
import rclpy
from rclpy.node import Node
from romi_msg.msg import RomiIn , RomiOut
import numpy as np
import time
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
from .quartenion import quaternion_from_euler, euler_from_quaternion
from .quartenion import quaternion_from_euler
from .quartenion import quaternion_from_euler, euler_from_quaternion
from tf2_msgs.msg import TFMessage
import math
from geometry_msgs.msg import Twist
from .naive_planner import NaivePlanner
from .tentacle_planner import TentaclePlanner

#PATH = [(0.5, 0.0), (0.0, 0.5), (-0.5, 0.0), (0.0, -0.5), (0.5, 0.0), (1.0, 0.0)]
#PATH = [(2.0,0.0)]
PATH = [(0.5, 0.0), (0.5, 0.5), (-0.5, 0.5), (-0.5, -0.5), (0.5, -0.5), (0.5, 0.0), (1.0, 0.0)]
"""
Handles the PID controller of the robot and i2c communication with Romi board
"""
EPSILON = 1e-6
ENCODER=12*3952/33


    
class DiffDriveRobot():
    """
    Adapted from ECE4191 code 
    
    """
    def __init__(self, inertia=5, drag=0.2, wheel_radius=0.035, wheel_sep=0.14):
        
        self.x = 0.0 # x-position
        self.y = 0.0 # y-position 
        self.th = 0.0 # orientation
        
        self.wl = 0.0 #rotational velocity left wheel
        self.wr = 0.0 #rotational velocity right wheel
        self.r = wheel_radius
        self.l = wheel_sep
        self.prev_t = time.time()

        self.prev_t=0
        self.prev_enc_l, self.prev_enc_r = 0.0, 0.0

        
        self.prev_enc_l=0
        self.prev_enc_r=0

        self.prev_enc_l, self.prev_enc_r = 0.0, 0.0

        
        self.v=0 #measured
        self.w=0 # measured
        
        
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
        
    
    # Kinematic motion model
    def pose_update(self,encoder_l,encoder_r, current_t):
    #def pose_update(self, wl, wr):
        
        dt = current_t-self.prev_t


        self.prev_t = current_t

        

        self.wl = min(max(2*np.pi*(self.overflow_corr(encoder_l, self.prev_enc_l))/dt/ENCODER, -12),12)
        self.wr = min(max(2*np.pi*(self.overflow_corr(encoder_r, self.prev_enc_r))/dt/ENCODER, -12),12)

        self.prev_enc_l=encoder_l
        self.prev_enc_r=encoder_r
        
        v, w = self.base_velocity(self.wl,self.wr)
        
        self.x = self.x + dt*v*np.cos(self.th)
        self.y = self.y + dt*v*np.sin(self.th)
        self.th = self.th + w*dt
        self.v=v
        self.w=w
        
        return self.x, self.y, self.th


class RobotController(DiffDriveRobot):
    """
    Adapted from ECE4191 code 
    
    """
    
    def __init__(self, dt=0.1, inertia=5, drag=0.2, wheel_radius=0.035, wheel_sep=0.14, Kp=2,Ki=0.9, Kd=5, exp_alpha=1):
        
        DiffDriveRobot.__init__(self,inertia, drag, wheel_radius, wheel_sep)
        self.Kp = Kp
        self.Kp_current = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.duty_cycle_l = 0
        self.duty_cycle_r = 0
        self.e_prev = 0
        self.I = 0

        self.exp_alpha = exp_alpha
        
    def p_control(self,duty_cycle, w_desired,w_measured):
        

        e = min(max(w_desired-w_measured, -200), 200)
        
        P = self.Kp*e
        self.I = self.I + self.Ki*e
        D = self.Kd*(e - self.e_prev)

        self.e_prev = e

        duty_cycle = self.exp_alpha*min(max(P + self.I + D, -200),200) + (1-self.exp_alpha)*duty_cycle
        return duty_cycle
        
        
    def drive(self,v_desired,w_desired,wl,wr):
        
        # print(f'wl desired {wl}')
        # print(f' wr desired {wr}')
        print(f'wl desired {wl}')
        print(f' wr desired {wr}')
        # print(f'wl desired {wl}')
        # print(f' wr desired {wr}')
        wl_desired = (v_desired - self.l*w_desired/2)/self.r
        wr_desired = (v_desired + self.l*w_desired/2)/self.r
        
        self.duty_cycle_l = self.p_control(self.duty_cycle_l, wl_desired,wl)*1.005
        self.duty_cycle_l = self.p_control(self.duty_cycle_l, wl_desired,wl)
        self.duty_cycle_l = self.p_control(self.duty_cycle_l, wl_desired,wl)*1.005
        self.duty_cycle_r = self.p_control(self.duty_cycle_r, wr_desired,wr)
        return self.duty_cycle_l, self.duty_cycle_r

        
class ControlNode(Node):

    def __init__(self):
        super().__init__('robot_control')

        self.declare_parameter('robot_id', '0')

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        self.a_star = AStar()

        self.robot = RobotController()
        self.publisher=self.create_publisher(RomiOut, 'robot_'+self.robot_id+'/status', 10)
        self._odom_pub=self.create_publisher(Odometry, 'robot_'+self.robot_id+'/odometry', 10)

        self.imu_pub = self.create_publisher(Imu, 'robot_'+self.robot_id+'/imu', 10)

        #self.cmd_vel_sub =  self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # self.subscription= self.create_subscription(
        #     RomiIn,
        #     'commander',
        #     self.listener_callback,
        #     1)

        # self.odom_sub = self.create_subscription(
        #         Odometry,
        #         'robot_'+self.robot_id+'/filtered_odom',
        #         self.odom_planner,
        #         10

        #     )

        # self.pose_subscription = self.create_subscription(
        #     Odometry,
        #     'robot_0/filtered_pose',
        #     self.planner_server,
        #     10
        # )
        self.subscription= self.create_subscription(
            RomiIn,
            'commander',
            self.listener_callback,
            10)
        #self.cmd_vel_sub =  self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # self.subscription= self.create_subscription(
        #     RomiIn,
        #     'commander',
        #     self.listener_callback,
        #     1)

        # self.odom_sub = self.create_subscription(
        #         Odometry,
        #         'robot_'+self.robot_id+'/filtered_odom',
        #         self.odom_planner,
        #         10

        #     )

        self.pose_subscription = self.create_subscription(
            Odometry,
            'robot_0/filtered_pose',
            self.planner_server,
            1)


        # self.planner = NaivePlanner('0')
        # self.robot_timer=self.create_timer(0.01, self.robot_timer_callback)
        # # self.timer = self.create_timer(0.2, self.robot_control)
        # self.robot_timer=self.create_timer(0.05, self.robot_timer_callback)
        self.planner = TentaclePlanner('0')
        self.robot_timer=self.create_timer(0.01, self.robot_timer_callback)
        # self.timer = self.create_timer(0.2, self.robot_control)

        self.desired_v=0
        self.desired_w=0
        self.x, self.y, self.th= 0, 0, 0
        self.prev_enc = []

        self.wx, self.wy, self.wz = 0,0,0
        self.done = 0

        self.odom_x, self.odom_y, self.odom_th = 0.0, 0.0, 0.0

        self.counter, self.temp_counter = 0, 0

        self.goal_x, self.goal_y, self.goal_th = 0.0, 0.0, 0.0

        self.start = 1
        self.prev_t = 0
        self.prev_enc_l = 0
        self.prev_enc_r = 0

    def base_velocity(self,wl,wr):

        r = 0.035
        l = 0.14

        v = (wl*r + wr*r)/2.0
        
        w = (wr*r - wl*r)/l #modified to adhre to REP103
        
        return v, w

    def overflow_corr(self, enc, prev_enc):

        val = enc - prev_enc
        if val > 32768:
            val = val - 32768
        
        elif val < -32768:

            val = val + 32768

        return val

    def read_encoders(self):

        while True:

            try:
                prev_enc_l, prev_enc_r = self.a_star.read_encoders()
                return prev_enc_l, prev_enc_r
            except:
                continue




    # def robot_control(self):
    #     PATH_T = [(2.0, 0.0, 0.0), (2.0, 0.0, 3.142)]
    #     r = 0.035


    #     if self.start:
    #         self.start = 0
    #         self.prev_t = time.time()
    #         self.prev_enc_l, self.prev_enc_r = self.read_encoders()

            

    #     else:
    #         current_t = time.time()
    #         dt = current_t-self.prev_t

    #         self.prev_t = current_t
    #         encoder_l, encoder_r = self.read_encoders()

            

    #         self.wl = min(max(2*np.pi*(self.overflow_corr(encoder_l, self.prev_enc_l))/dt/ENCODER, -12),12)
    #         self.wr = min(max(2*np.pi*(self.overflow_corr(encoder_r, self.prev_enc_r))/dt/ENCODER, -12),12)

    #         self.prev_enc_l=encoder_l
    #         self.prev_enc_r=encoder_r
            
    #         v, w = self.base_velocity(self.wl,self.wr)
            
    #         self.x = self.x + dt*v*np.cos(self.th)
    #         self.y = self.y + dt*v*np.sin(self.th)
    #         print(f'x: {self.x}')
    #         print(f'y: {self.y}')
    #         print(f'th: {self.th}')
    #         self.th = (self.th + w*dt) % (3.142*2)
    #         self.v=v
    #         self.w=w

    #         dist = np.sqrt((PATH_T[self.counter][0]-self.x)**2+(PATH_T[self.counter][1]-self.y)**2)
    #         e_th = PATH_T[self.counter][2] - self.th
    #         e_th = np.arctan2(np.sin(e_th), np.cos(e_th))
    #         print(f'e_th: {e_th}')

    #         print(self.counter)
    #         while True:

    #             try:
    #                 if (dist < 0.1 and abs(e_th)<0.08) or self.done:
    #                     self.motors(0.0, 0.0)
    #                     self.counter = self.counter+1


    #                     if self.counter == len(PATH_T):
    #                         self.counter = len(PATH_T) - 1
    #                         self.done = 1



    #                 elif self.counter == 0:
    #                     duty_cycle_l,duty_cycle_r = self.robot.drive(0.4, 0.0, self.wl, self.wr)
    #                     self.motors(duty_cycle_l, duty_cycle_r)

    #                 elif self.counter == 1:
                        
    #                     duty_cycle_l,duty_cycle_r = self.robot.drive(0.0, 2.0, self.wl, self.wr)
    #                     print(duty_cycle_l)
    #                     print(duty_cycle_r)
    #                     self.motors(-50, 50)


    #                 break

    #             except:
    #                 print("I/O error")
    #                 continue


    def cmd_vel_callback(self, msg:Twist):

        self.desired_v = msg.linear.x
        self.desired_w = msg.angular.z




        duty_cycle_l,duty_cycle_r = self.robot.drive(self.desired_v, self.desired_w, self.robot.wl, self.robot.wr)
        self.motors(duty_cycle_l, duty_cycle_r)


    def planner_server(self, msg:Odometry):
        
        # self.get_logger().info('v: "%s" ' % str(self.desired_v))
        # self.get_logger().info('w: "%s" ' % str(self.desired_w))
        if self.counter == len(PATH):
            
            self.desired_v,self.desired_w  = 0.0, 0.0

        else:
            dx = PATH[self.counter][0] - msg.pose.pose.position.x
            dy = PATH[self.counter][1] - msg.pose.pose.position.y
            goal_th = np.arctan2(dy, dx)

            euler = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
            #self.get_logger().info('th: "%s"' %str(self.counter))
            self.desired_v,self.desired_w = self.planner.plan(PATH[self.counter][0], PATH[self.counter][1], goal_th, msg.pose.pose.position.x, msg.pose.pose.position.y, euler[2])

        if self.desired_v == 0.0 and self.desired_w ==0.0 and self.counter<len(PATH):
            

            self.get_logger().info('Reached x: "%s" ' % str(PATH[self.counter][0]))
            self.get_logger().info('Reached y: "%s" ' % str(PATH[self.counter][1]))
            self.get_logger().info('th: "%s"' %str(self.counter))
            self.counter += 1


        if self.desired_v == 0.0 and self.desired_w == 0.0:
            self.motors(0,0)
            time.sleep(0.1)

        else:
            duty_cycle_l,duty_cycle_r = self.robot.drive(self.desired_v, self.desired_w, self.robot.wl, self.robot.wr)
            self.motors(duty_cycle_l, duty_cycle_r)

        


        

        
    def robot_timer_callback(self):

        while True:
            try:
                encoders = self.a_star.read_encoders()
                self.wx, self.wy, self.wz = self.a_star.read_gyroscope() 

                
                break
            except Exception as e:
                self.get_logger().info('"%s"' % str(e))


                continue

        # try:
        #     encoders = self.a_star.read_encoders()
        #     self.wx, self.wy, self.wz = self.a_star.read_gyroscope()
        #     self.x, self.y, self.th = self.robot.pose_update(encoders[0], encoders[1], time.time()) 
        
        # except:
        #     self.get_logger().info("I/O error")
        
    
        #self.get_logger().info('time: "%s"' %str(time.time()))     
             
        #self.get_logger().info('time: "%s"' %str(time.time()))     
        

        t = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.frame_id = 'robot_'+self.robot_id+'/odom'
        odom.header.stamp = t
        odom.child_frame_id ='robot_'+self.robot_id
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = float(0)
        q = quaternion_from_euler(.0, .0, self.robot.th)
        odom.pose.pose.orientation.x = float(q[0])
        odom.pose.pose.orientation.y = float(q[1])
        odom.pose.pose.orientation.z = float(q[2])
        odom.pose.pose.orientation.w = float(q[3])
        odom.pose.covariance = np.diag([1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1]).ravel()
        odom.twist.twist.linear.x = float(self.robot.v)
        odom.twist.twist.angular.z = float(self.robot.w)
        odom.twist.covariance = np.diag([1e-5, 1e-5, 1e-5, 1e-6, 1e-6, 1e-6]).ravel()
        odom.twist.covariance = np.diag([1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3]).ravel()
        odom.twist.covariance = np.diag([1e-5, 1e-5, 1e-5, 1e-6, 1e-6, 1e-6]).ravel()
        self._odom_pub.publish(odom) 

        imu = Imu()
        imu.header.frame_id = 'robot_'+self.robot_id+'/odom'
        imu.header.stamp = t

        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = 0.0,0.0,0.0,0.0
        imu.orientation_covariance = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]

        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = float(self.wx), float(self.wy), float(self.wz)
        imu.angular_velocity_covariance = [1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4]

        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = 0.0,0.0,0.0
        imu.linear_acceleration_covariance = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]

        self.imu_pub.publish(imu)


        duty_cycle_l,duty_cycle_r = self.robot.drive(self.desired_v, self.desired_w, self.robot.wl, self.robot.wr)


        self.motors(duty_cycle_l, duty_cycle_r)

       


    def listener_callback(self, msg):

        if self.desired_v != msg.v or self.desired_w != msg.w:
            self.robot.Kp_current = self.robot.Kp

        self.desired_v=msg.v
        self.desired_w=msg.w


    def status(self):
        while True:
            try:
                battery_millivolts = float(self.a_star.read_battery_millivolts()[0])
                
                break
            except:
                print("I/O error")
             
                continue
        
        data = [battery_millivolts, float(self.robot.wr),  float(self.robot.wl), float(self.x), float(self.y), float(self.th)]

        return data


    def motors(self,left, right):

        while True:
            try:
                self.a_star.motors(int(left), int(right))
                return
            except:

                self.get_logger().info("I/O error")
                print("I/O error")
                self.get_logger().info("I/O error")

                continue



def main(args=None):
    rclpy.init(args=args)
    robot=ControlNode()
    print("Starting...\n")
    rclpy.spin(robot) 
    rclpy.shutdown()
    print("Finished\n")

if __name__ == '__main__':
    #initalize the processes
    main()

# from .a_star import AStar
# import rclpy
# from rclpy.node import Node
# from romi_msg.msg import RomiIn , RomiOut
# import numpy as np
# import time
# import matplotlib.pyplot as plt
# from nav_msgs.msg import Odometry
# import math
# """
# Handles the PID controller of the robot and i2c communication with Romi board
# """
# EPSILON = 1e-6
# ENCODER=12*3952/33

# def quaternion_from_euler(ai, aj, ak):
#     ai /= 2.0
#     aj /= 2.0
#     ak /= 2.0
#     ci = math.cos(ai)
#     si = math.sin(ai)
#     cj = math.cos(aj)
#     sj = math.sin(aj)
#     ck = math.cos(ak)
#     sk = math.sin(ak)
#     cc = ci*ck
#     cs = ci*sk
#     sc = si*ck
#     ss = si*sk

#     q = np.empty((4, ))
#     q[0] = cj*sc - sj*cs
#     q[1] = cj*ss + sj*cc
#     q[2] = cj*cs - sj*sc
#     q[3] = cj*cc + sj*ss

#     return q
    
# class DiffDriveRobot():
#     """
#     Adapted from ECE4191 code 
    
#     """
#     def __init__(self, inertia=5, drag=0.2, wheel_radius=0.035, wheel_sep=0.14):
        
#         self.x = 0.0 # x-position
#         self.y = 0.0 # y-position 
#         self.th = 0.0 # orientation
        
#         self.wl = 0.0 #rotational velocity left wheel
#         self.wr = 0.0 #rotational velocity right wheel
#         self.r = wheel_radius
#         self.l = wheel_sep
#         self.prev_t = time.time()

#         self.prev_t=0
#         self.prev_enc_l=0
#         self.prev_enc_r=0

#         self.v=0 #measured
#         self.w=0 # measured
        
        
#     # Veclocity motion model
#     def base_velocity(self,wl,wr):
#         print(wl)
#         print(wr)
#         v = (wl*self.r + wr*self.r)/2.0
        
#         w = (wr*self.r - wl*self.r)/self.l #modified to adhre to REP103
        
#         return v, w

#     def overflow_corr(self, enc, prev_enc):

#         val = enc - prev_enc
#         if val > 32768:
#             val = val - 32768
        
#         elif val < -32768:

#             val = val + 32768

#         return val
        
    
#     # Kinematic motion model
#     def pose_update(self,encoder_l,encoder_r, current_t):
#     #def pose_update(self, wl, wr):
        
#         dt = current_t-self.prev_t
#         self.prev_t = current_t

        

#         self.wl = min(max(2*np.pi*(self.overflow_corr(encoder_l, self.prev_enc_l))/dt/ENCODER, -12),12)
#         self.wr = min(max(2*np.pi*(self.overflow_corr(encoder_r, self.prev_enc_r))/dt/ENCODER, -12),12)

#         self.prev_enc_l=encoder_l
#         self.prev_enc_r=encoder_r
        
#         v, w = self.base_velocity(self.wl,self.wr)
        
#         self.x = semp/launch_params_diyiof_w -r image_raw:=robot_0/image_raw -r set_capture:=robot_0/cam_3/set_capture -r camera_info:=robot_0/camera_info -r rosout:=robot_0/rosout/cam'].

#         self.x = self.x + dt*v*np.cos(self.th)
#         self.x = semp/launch_params_diyiof_w -r image_raw:=robot_0/image_raw -r set_capture:=robot_0/cam_3/set_capture -r camera_info:=robot_0/camera_info -r rosout:=robot_0/rosout/cam'].

#         self.y = self.y + dt*v*np.sin(self.th)
#         self.th = self.th + w*dt
#         self.v=v
#         self.w=w
        
#         return self.x, self.y, self.th


# class RobotController(DiffDriveRobot):
#     """
#     Adapted from ECE4191 code 
    
#     """
    
#     def __init__(self, dt=0.1, inertia=5, drag=0.2, wheel_radius=0.035, wheel_sep=0.14, Kp=2,Ki=0.9, Kd=5, exp_alpha=1):
        
#         DiffDriveRobot.__init__(self,inertia, drag, wheel_radius, wheel_sep)
#         self.Kp = Kp
#         self.Kp_current = Kp
#         self.Ki = Ki
#         self.Kd = Kd
#         self.duty_cycle_l = 0
#         self.duty_cycle_r = 0
#         self.e_prev = 0
#         self.I = 0

#         self.exp_alpha = exp_alpha
        
#     def p_control(self,duty_cycle, w_desired,w_measured):
        

#         e = min(max(w_desired-w_measured, -200), 200)
        
#         P = self.Kp*e
#         self.I = self.I + self.Ki*e
#         D = self.Kd*(e - self.e_prev)

#         self.e_prev mp/launch_params_diyiof_w -r image_raw:=robot_0/image_raw -r set_capture:=robot_0/cam_3/set_capture -r camera_info:=robot_0/camera_info -r rosout:=robot_0/rosout/cam'].

#         self.e_prev = e

#         duty_cycle = self.exp_alpha*min(max(P + self.I + D, -200),200) + (1-self.exp_alpha)*duty_cycle
#         return duty_cycle
        
        
#     def drive(self,v_desired,w_desired,wl,wr):
        
#         self.e_prev mp/launch_params_diyiof_w -r image_raw:=robot_0/image_raw -r set_capture:=robot_0/cam_3/set_capture -r camera_info:=robot_0/camera_info -r rosout:=robot_0/rosout/cam'].

#         print(f'wl desired {wl}')
#         print(f' wr desired {wr}')
#         wl_desired = (v_desired + self.l*w_desired/2)/self.r
#         wr_desired = (v_desired - self.l*w_desired/2)/self.r
        
#         self.duty_cycle_l = self.p_control(self.duty_cycle_l, wl_desired,wl)
#         self.duty_cycle_r = self.p_control(self.duty_cycle_r, wr_desired,wr)
        
#         return self.duty_cycle_l, self.duty_cycle_r

        
# class ControlNode(Node):

#     def __init__(self, robotID):
#         super().__init__('publisher_'+str(robotID))

#         self.declare_parameter('robot_id', '0')

#         self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

#         self.a_star = AStar()
#         self.robotID=robotID
#         self.robot = RobotController()
#         self.publisher=self.create_publisher(RomiOut, 'status_robot_'+str(self.robotID), 1)
#         self._odom_pub=self.create_publisher(Odometry, 'robot_'+self.robot_id+'/odometry', 10)
#         self.subscription= self.create_subscription(
#             RomiIn,
#             'commander',
#             self.listener_callback,
#             1)
#         timer_period=0.1
#         self.timer=self.create_timer(timer_period, self.timer_callback)

#         self.robot_timer=self.create_timer(0.001, self.robot_timer_callback)

#         self.desired_v=0
#         self.desired_w=0
#         self.x, self.y, self.th= 0, 0, 0
#         self.prev_enc = []
        

#     def timer_callback(self):
#         msg = RomiOut()
#         msg.robot_id = self.robotID
#         msg.romi_status = self.status()
#         t = self.get_clock().now().to_msg()
#         msg.timestamp= float(time.time())


#         self.publisher.publish(msg)


#         odom = Odometry()
#         odom.header.frame_id = 'robot_'+self.robot_id+'/odom'
#         odom.header.stamp = t
#         odom.child_fmp/launch_params_diyiof_w -r image_raw:=robot_0/image_raw -r set_capture:=robot_0/cam_3/set_capture -r camera_info:=robot_0/camera_info -r rosout:=robot_0/rosout/cam'].

#         odom.child_frame_id ='robot_'+self.robot_id
#         odom.child_fmp/launch_params_diyiof_w -r image_raw:=robot_0/image_raw -r set_capture:=robot_0/cam_3/set_capture -r camera_info:=robot_0/camera_info -r rosout:=robot_0/rosout/cam'].

#         odom.pose.pose.position.x = float(self.x)
#         odom.pose.pose.position.y = float(self.y)
#         odom.pose.pose.position.z = float(0)
#         q = quaternion_from_euler(.0, .0, self.robot.th)
#         odom.pose.pose.orientation.x = float(q[0])
#         odom.pose.pose.orientation.y = float(q[1])
#         odom.pose.pose.orientation.z = float(q[2])
#         odom.pose.pose.orientation.w = float(q[3])
#         odom.pose.covariance = np.diag([1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1]).ravel()
#         odom.twist.twist.linear.x = float(self.robot.v)
#         odom.twist.twist.angular.z = float(self.robot.w)
#         odom.twist.covariance = np.diag([1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3]).ravel()
#         self._odom_pub.publish(odom) 


#     def robot_timer_callback(self):

#         while True:
#             try:
#                 encoders = self.a_star.read_encoders()
                
#                 break
#             except:
#                 print("I/O error")

#                 continue
        
    
             
#         self.x, self.y, self.th = self.robot.pose_update(encoders[0], encoders[1], time.time())
#         duty_cycle_l,duty_cycle_r = self.robot.drive(self.desired_v, self.desired_w, self.robot.wl, self.robot.wr)

#         self.motors(duty_cycle_l, duty_cycle_r)
       

#     def listener_callback(self, msg):

#         if self.desired_v != msg.v or self.desired_w != msg.w:
#             self.robot.Kp_current = self.robot.Kp

#         self.desired_v=msg.v
#         self.desired_w=msg.w

#     def status(self):
#         while True:
#             try:
#                 battery_millivolts = float(self.a_star.read_battery_millivolts()[0])
                
#                 break
#             except:
#                 print("I/O error")
             
#                 continue
        
#         data = [battery_millivolts, float(self.robot.wr),  float(self.robot.wl), float(self.x), float(self.y), float(self.th)]

#         return data


#     def motors(self,left, right):

#         while True:
#             try:
#                 self.a_star.motors(int(left), int(right))
#                 return
#             except:

#                 print("I/O error")

#                 continue



# def main(args=None):
#     rclpy.init(args=args)/home/ubuntu/github_repo/networked_robotics/apriltags_localization/src/apriltags_localization/launch/raspi_nodes.launch.py
#     rclpy.init(args=args)
#     rclpy.init(args=args)/home/ubuntu/github_repo/networked_robotics/apriltags_localization/src/apriltags_localization/launch/raspi_nodes.launch.py
#     robot=ControlNode(1)
#     print("Starting...\n")
#     rclpy.spin(robot) 
#     rclpy.shutdown()
#     print("Finished\n")

# if __name__ == '__main__':
#     #initalize the processes
#     main()

