from std_srvs.srv import SetBool
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time
from rcl_interfaces.msg import Log
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
import numpy as np
"""
This code handles the polling of the cameras
"""

class Poller(Node):

    def __init__(self):
        
        super().__init__('robot_poller')

        self.declare_parameter('robot_id', '0')

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        client_cb_group = MutuallyExclusiveCallbackGroup()
        timer_cb_group = None
        
        self.cli_0 = self.create_client(SetBool, 'robot_'+self.robot_id+'/cam_0/set_capture', callback_group=client_cb_group)
        # self.cli_1 = self.create_client(SetBool, 'robot_'+self.robot_id+'/cam_1/set_capture', callback_group=client_cb_group)
        # self.cli_2 = self.create_client(SetBool, 'robot_'+self.robot_id+'/cam_2/set_capture', callback_group=client_cb_group)
        # self.cli_3 = self.create_client(SetBool, 'robot_'+self.robot_id+'/cam_3/set_capture', callback_group=client_cb_group)

        # self.cli_list = [self.cli_0, self.cli_1, self.cli_2, self.cli_3]
        self.cli_list = [self.cli_0]

        while not self.cli_0.wait_for_service(timeout_sec=1.0):
        # or not self.cli_1.wait_for_service(timeout_sec=1.0) \
        # or not self.cli_2.wait_for_service(timeout_sec=1.0) \
        # or not self.cli_3.wait_for_service(timeout_sec=1.0) :
            self.get_logger().info('service not available, waiting again...')


        
        self.det = self.create_subscription(
            TFMessage,
            'robot_'+self.robot_id+'/tf_detections',
            self.listener_callback,
            10)


        self.pose_sub = self.create_subscription(
            Odometry,
            'robot_'+str(self.robot_id)+'/filtered_pose',
            self.poll_dir_update,
            10
        )


        self.on_req = SetBool.Request()
        self.off_req = SetBool.Request()

        self.on_req.data = True
        self.off_req.data = False

        self.pair = 0
        self.prev_pair = 0

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.send_request, callback_group=timer_cb_group)

        self.detected = 0
        self.switch = 1
        self.dir = 1
        self.run = 0

        self.query_pose = False

    def poll_dir_update(self, msg):

        self.query_pose = True
        euler = R.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]).as_euler('xyz')
        robot_pose = np.arctan2(msg.pose.pose.position.y, msg.pose.pose.position.x)

        angle = (np.pi - (robot_pose - euler[2]) + np.pi) % (2*np.pi) - np.pi


        if angle > 0:
            if angle < 0.5*1.571:
                self.pair = 0

            elif angle > 1.5*1.571:
                self.pair = 2

            else:
                self.pair = 3

        else:
            if angle > 0.5*-1.571:
                self.pair = 0

            elif angle < -1.5*1.571:
                self.pair = 2

            else:
                self.pair = 1


    def listener_callback(self, msg):
        self.run = 1

        if msg.transforms:

            self.detected = 0
        
        else:
            self.detected-=1

        if self.detected < -3:
            self.switch = 1
        else:
            self.switch = 0


    def send_request(self):

        if self.switch or not self.run:
            self.detected = 0
            self.run = 0

            while not self.cli_list[self.pair].wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')

       
            on = self.cli_list[self.pair].call_async(self.on_req)
            
            
            if self.pair != self.prev_pair:

                while not self.cli_list[self.prev_pair].wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('service not available, waiting again...')
                off = self.cli_list[self.prev_pair].call_async(self.off_req)

            self.prev_pair = self.pair

            if not self.query_pose:
                self.pair = (self.pair + self.dir) % 4

            self.switch = 0
           



        


def main():
    rclpy.init()

    minimal_client = Poller()
    rclpy.spin(minimal_client)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()