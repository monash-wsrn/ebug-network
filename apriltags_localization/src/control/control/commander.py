import rclpy
from rclpy.node import Node
from romi_msg.msg import RomiIn , RomiOut
import time
import matplotlib.pyplot as plt
import numpy as np
from .tentacle_planner import TentaclePlanner
from obstacles_msg.msg import ObstaclesArray, Obstacles
from .obstacle import Obstacle as Obs
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from nav_msgs.msg import Odometry
from .quartenion import euler_from_quaternion

"""
This code give veolcity commands to drive the robot
"""


#PATH = [(0.5, 0.0), (0.0, 0.5), (-0.5, 0.0), (0.0, -0.5), (0.5, 0.0), (1.0, 0.0)]
PATH = [(0.5, 0.0), (0.5, 0.5), (-0.5, 0.5), (-0.5, -0.5), (0.5, -0.5), (0.5, 0.0)]
#PATH = [(0.5, 0.0, 3.142), (-0.5, 0.0, 3.142), (0.5, 0.0, 0.0)]

ROBOT_ID = '0'

class CommanderNode(Node):

    def __init__(self):
        super().__init__('commander')

        self.declare_parameter('show_plot', '0')

        self.show_plot = self.get_parameter('show_plot').get_parameter_value().integer_value
        self.robotID = 0
        self.publisher=self.create_publisher(RomiIn, 'commander',1)
        self.subscription= self.create_subscription(
            RomiOut,
            'robot_'+str(self.robotID)+'/status',
            self.listener_callback,
            1)
        timer_period=1

        self.obs_subscription = self.create_subscription(
            ObstaclesArray,
            'obstacles',
            self.update_obstacles,
            1)

        self.pose_subscription = self.create_subscription(
            Odometry,
            'robot_0/filtered_pose',
            self.planner_server,
            1)

        #self.timer=self.create_timer(timer_period, self.timer_callback)
        self.v=0
        self.w=0
        self.v_list=np.array([])
        self.w_list=np.array([])
        self.counter=0

        self.planner = TentaclePlanner(ROBOT_ID,[])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.temp_counter = 0






    # def planner_server(self, msg:Odometry):
        
        
        

    #     if self.counter == len(PATH):
            
    #         v, w = 0.0, 0.0

    #     else:
    #         dx = PATH[self.counter][0] - msg.pose.pose.position.x
    #         dy = PATH[self.counter][1] - msg.pose.pose.position.y
    #         goal_th = np.arctan2(dy, dx)

    #         euler = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

    #         v,w = self.planner.plan(PATH[self.counter][0], PATH[self.counter][1], goal_th, msg.pose.pose.position.x, msg.pose.pose.position.y, euler[2])

        if v == 0.0 and w ==0.0 and self.counter<len(PATH):

            self.temp_counter+=1

        elif self.counter<len(PATH):
            self.temp_counter = 0

        if self.temp_counter >= 30:
            self.get_logger().info('Reached x: "%s" ' % str(PATH[self.counter][0]))
            self.get_logger().info('Reached y: "%s" ' % str(PATH[self.counter][1]))
            self.counter += 1
            self.temp_counter = 0

        

    #     msg = RomiIn()
    #     msg.robot_id=0
    #     msg.v=float(v)
    #     msg.w=float(w)
    #     msg.timestamp=time.time()

    #     print('check')
    #     self.publisher.publish(msg)

        if v == 0.0 and w ==0.0:
            time.sleep(0.1)

        

        

       


    def obstacles_msg_to_obstacles(self, obstacles_msg):
        obs = Obs(obstacles_msg.name, obstacles_msg.x, obstacles_msg.y, [], obstacles_msg.prohibited_rad,0.999)

        return obs

    def update_obstacles(self, msg):

        arr = []
        for i in msg.obstacles_array:
            arr.append(self.obstacles_msg_to_obstacles(i))

        # del arr[int(ROBOT_ID)]
        self.planner.obstacles = arr
        

    # def timer_callback(self):
    #     msg = RomiIn()
    #     self.motor_controller()
    #     msg.robot_id=1
    #     msg.v=float(self.v)
    #     msg.w=float(self.w)
    #     msg.timestamp=time.time()


    #     self.publisher.publish(msg)

    def listener_callback(self, msg):

        self.v_list=np.append(self.v_list,msg.romi_status[2])
        self.w_list=np.append(self.w_list, msg.romi_status[1])
        
        
    # def motor_controller(self):

    #     valid = False
        
    #     while not valid:

    #         self.v=0.1
    #         self.w=0.0

    #         if abs(self.v)<=20 and abs(self.w)<=20:
    #             valid = True       

    #         if self.counter==10:
    #             self.counter=0
                
    #             valid = True 

    #             if self.show_plot:
    #                 plt.subplot(1,2,1)
    #                 plt.plot(self.w_list)
    #                 plt.title("w left")
    #                 plt.subplot(1,2,2)
    #                 plt.plot(self.v_list)
    #                 plt.title("w right")
    #                 plt.show()
    #                 self.v_list=np.array([])
    #                 self.w_list=np.array([])

    #         self.counter+=1



def main(args=None):
    rclpy.init(args=args)
    commander=CommanderNode()
    print("Starting...\n")
    rclpy.spin(commander) 
    rclpy.shutdown()
    print("Finished\n")

if __name__ == '__main__':
    #initalize the processes
    main()