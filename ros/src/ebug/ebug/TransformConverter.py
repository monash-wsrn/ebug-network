
import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped, Transform

import math


# CAM_0 = (0, 0,     0) RPY Radians (FRONT)
# CAM_1 = (0, 0,  PI/2) RPY Radians (RIGHT)
# CAM_2 = (0, 0,    PI) RPY Radians (BACK)
# CAM_3 = (0, 0, 3PI/2) RPY Radians (LEFT)

CAM_OFFSET = 0.025
CAM_ROTATION = [
    0.0,                    # Camera 0: Forward
    math.pi / 2.0,          # Camera 1: Right
    math.pi,                # Camera 2: Behind
    3.0 * math.pi / 2.0     # Camera 3: Left
]

"""
This code inverse the cam->tag transform
"""

class TransformConverter(Node):

    def __init__(self):
        super().__init__(self.__class__.__name__)

        self.subscription = self.create_subscription(TFMessage, 'tf_detections', self.listener_callback, 100)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, 'pose', 100)

        self.covariance = mat6diag(1e-1)


    def listener_callback(self, tf_det:TFMessage):
        for t in tf_det.transforms:
            cam_id = int(t.header.frame_id[-1])

            distance = t.transform.translation.z + CAM_OFFSET
            rotation = quat2roll(t.transform.rotation) + CAM_ROTATION[cam_id]

            msg = PoseWithCovarianceStamped()
            msg.header = t.header
            msg.header.frame_id = t.child_frame_id
            
            msg.pose.pose.position.x = 0.0
            msg.pose.pose.position.y = 0.0
            msg.pose.pose.position.z = -distance

            qx, qy, qz, qw = roll2quat(rotation + math.pi)
            msg.pose.pose.orientation.x = qx
            msg.pose.pose.orientation.y = qy
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw

            msg.pose.covariance = self.covariance
            self.publisher.publish(msg)


def mat6diag(v):
    return [(float(v) if i % 7 == 0 else 0.0) for i in range(36)]


# Function to get the roll (radians) of a quaternion
def quat2roll(quat):
    x, y, z, w = quat.x, quat.y, quat.z, quat.w

    # https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    return math.atan2(t0, t1)

# Function to get the quaternion of a roll (radians)
def roll2quat(roll):
    pitch, yaw = 0.0, 0.0
    
    # https://stackoverflow.com/q/53033620
    qx = math.sin(roll/2.0) * math.cos(pitch/2.0) * math.cos(yaw/2.0) - math.cos(roll/2.0) * math.sin(pitch/2.0) * math.sin(yaw/2.0)
    qy = math.cos(roll/2.0) * math.sin(pitch/2.0) * math.cos(yaw/2.0) + math.sin(roll/2.0) * math.cos(pitch/2.0) * math.sin(yaw/2.0)
    qz = math.cos(roll/2.0) * math.cos(pitch/2.0) * math.sin(yaw/2.0) - math.sin(roll/2.0) * math.sin(pitch/2.0) * math.cos(yaw/2.0)
    qw = math.cos(roll/2.0) * math.cos(pitch/2.0) * math.cos(yaw/2.0) + math.sin(roll/2.0) * math.sin(pitch/2.0) * math.sin(yaw/2.0)
    return [qx, qy, qz, qw]
                


def main():
    rclpy.init()
    node = TransformConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        return
    finally:
        node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()