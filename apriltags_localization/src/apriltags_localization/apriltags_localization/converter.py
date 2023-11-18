import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
from .robot_pose import *
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from apriltag_msgs.msg import AprilTagDetectionArray
from control.quartenion import euler_from_quaternion


"""
This code inverse the cam->tag transform
"""
bridge = CvBridge()

class Converter(Node):

    def __init__(self):
        
        super().__init__('robot_converter')

        self.declare_parameter('robot_id', '0')

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        self.declare_parameter('store_image', 0)

        self.capture = self.get_parameter('store_image').get_parameter_value().integer_value

        self.declare_parameter('im_path', 'im_save')

        self.im_path = self.get_parameter('im_path').get_parameter_value().string_value

        self.alpha = 0.05
        self.beta = 0.2

        self.subscription = self.create_subscription(
            TFMessage,
            'robot_'+self.robot_id+'/tf_detections',
            self.listener_callback,
            100)

        if self.capture:
            self.image_subscription = self.create_subscription(
                Image,
                'cam_0/image_raw',
                self.cam_callback,
                1)

        self.counter = 0

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, 'robot_'+self.robot_id+'/pose', 100)

    def cam_callback(self, msg):
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite(self.im_path+'/im_'+str(msg.header.stamp)+'.jpg', cv2_img)


    def listener_callback(self, tf_det:TFMessage):
        # Store frame names in variables that will be used to
        # compute transformations
        



        if tf_det.transforms:

            for t in tf_det.transforms:
                euler = euler_from_quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,t.transform.rotation.w)

                # if abs(euler[2])> np.radians(50):
                #     return
                
                tag_cam = inverseTransform(t.transform)


                tag_robot = tag_to_robot(tag_cam, get_robot_cam_transform(int(t.header.frame_id[-1])))

                msg = PoseWithCovarianceStamped()

                msg.header = t.header
                msg.header.frame_id = 'apriltag_' + t.child_frame_id[-1]
                msg.pose.pose = convert_translation_rotation_to_pose(tag_robot.translation, tag_robot.rotation)

                msg.pose.pose.orientation.y = -msg.pose.pose.orientation.y 
                #covariance
                # Row-major representation of the 6x6 covariance matrix
                # The orientation parameters use a fixed-axis representation.
                # In order, the parameters are:
                # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)

                # dist = math.sqrt((msg.pose.pose.position.x ** 2) + (msg.pose.pose.position.y ** 2) + (msg.pose.pose.position.z ** 2))
                # off_angle_scaling = np.exp((0.5 - abs(t.transform.rotation.w)))

                # counter_scale = 3/(self.counter+1)
                # # * self.beta * 1/(self.counter+self.beta)
                # covar_scale = dist * off_angle_scaling*counter_scale 

                # covar = np.diag([self.alpha * covar_scale, self.alpha*covar_scale, self.alpha*covar_scale, self.alpha*covar_scale, self.alpha*covar_scale, self.alpha*covar_scale]).flatten()
                # covar = [float(i) for i in covar]

                # msg.pose.covariance = covar
                self.publisher.publish(msg)

            self.counter += 1

        else:
            self.counter = 0


def main():
    rclpy.init()
    node = Converter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()