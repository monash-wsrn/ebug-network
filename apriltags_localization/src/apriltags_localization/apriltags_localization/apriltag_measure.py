import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf2_msgs.msg import TFMessage
from .robot_pose import *
import numpy as np
from .tf_transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import math

"""
This code is to measure the raw detection result and plot it out,
"""

class Measure(Node):

    def __init__(self):
        super().__init__('measure')

        self.subscription = self.create_subscription(
            TFMessage,
            'robot_0/tf_detections',
            self.listener_callback,
            100)

        self.x = []
        self.y = []


        self.theta =[]

        self.plot_timer=self.create_timer(30, self.plot_callback)


    def quartenion_to_list(self, q: Quaternion):

        return [q.x, q.y, q.z, q.w]



    def listener_callback(self, msg: TFMessage):

        if msg.transforms:
            self.x.append(msg.transforms[0].transform.translation.z)
            self.y.append(msg.transforms[0].transform.translation.x)

            euler = euler_from_quaternion(self.quartenion_to_list(msg.transforms[0].transform.rotation))

            self.theta.append(euler[1])



    def plot_callback(self):

        u = 1*np.cos(self.theta)
        v = 1*np.sin(self.theta)

        plt.figure(1)
        plt.quiver(self.x,self.y, u, v)
        plt.xlabel('distance straight out of cam to tag (m)')
        plt.ylabel('distance in positive-right direction (m)')

        plt.figure(2)
        plt.plot(self.x, np.degrees(self.theta), 'x')
        plt.ylabel('theta (degrees)')
        plt.xlabel('distance straight out of cam to tag (m)')

        self.x= []
        self.y = []
        self.theta = []
        plt.show()



def main():
    rclpy.init()
    node = Measure()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()



