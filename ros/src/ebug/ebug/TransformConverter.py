
import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped, Transform



# CAM_0 = (0, 0,     0) RPY Radians (FRONT)
# CAM_1 = (0, 0,  PI/2) RPY Radians (RIGHT)
# CAM_2 = (0, 0,    PI) RPY Radians (BACK)
# CAM_3 = (0, 0, 3PI/2) RPY Radians (LEFT)

CAM_ROBOT_ROT_X = [0.0,        0.0,  0.0,         0.0]
CAM_ROBOT_ROT_Y = [0.0,        0.0,  0.0,         0.0]
CAM_ROBOT_ROT_Z = [0.0,  0.7071068,  1.0,   0.7071068]
CAM_ROBOT_ROT_W = [0.0,  0.7071068,  0.0,  -0.7071068]

"""
This code inverse the cam->tag transform
"""

class TransformConverter(Node):

    def __init__(self):
        super().__init__(self.__class__.__name__)


        self.subscription = self.create_subscription(TFMessage, 'tf_detections', self.listener_callback, 100)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, 'pose', 100)

        self.cameras = [self.get_camera(i) for i in range(4)]
        self.covariance = mat6diag(1e-3)
    
    def get_camera(self, cam_id):
        t = Transform()

        t.translation.x = 0.0
        t.translation.y = 0.0
        t.translation.z = -0.025

        t.rotation.x = CAM_ROBOT_ROT_X[cam_id]
        t.rotation.y = CAM_ROBOT_ROT_Y[cam_id]
        t.rotation.z = CAM_ROBOT_ROT_Z[cam_id]
        t.rotation.w = CAM_ROBOT_ROT_W[cam_id]
        
        return t


    def listener_callback(self, tf_det:TFMessage):
        if not tf_det.transforms:
            return
        
        for t in tf_det.transforms:    
            cam_id = int(t.child_frame_id[-1])

            tag_cam = inverse(t.transform)
            cam_robot = self.cameras[ cam_id ]
            tag_robot = combine(tag_cam, cam_robot)
            robot_tag = inverse(tag_robot)

            msg = PoseWithCovarianceStamped()
            msg.header = t.header
            msg.header.frame_id = f'apriltag_{cam_id}'
            
            msg.pose.pose.position.x = robot_tag.translation.x
            msg.pose.pose.position.y = robot_tag.translation.y
            msg.pose.pose.position.z = robot_tag.translation.z

            msg.pose.pose.orientation.x = robot_tag.rotation.x
            msg.pose.pose.orientation.y = robot_tag.rotation.y
            msg.pose.pose.orientation.z = robot_tag.rotation.z
            msg.pose.pose.orientation.w = robot_tag.rotation.w

            msg.pose.covariance = self.covariance
            self.publisher.publish(msg)


def combine(t1, t2):
    t = Transform()

    t.translation.x = t1.translation.x + t2.translation.x
    t.translation.y = t1.translation.y + t2.translation.y
    t.translation.z = t1.translation.z + t2.translation.z

    qx, qy, qz, qw = mulQuat(t1.rotation, t2.rotation)
    t.rotation.x = qx
    t.rotation.y = qy
    t.rotation.z = qz
    t.rotation.w = qw
    
    return t


def inverse(t1):    
    t = Transform()

    t.translation.x = -t1.translation.x
    t.translation.y = -t1.translation.y
    t.translation.z = -t1.translation.z

    qx, qy, qz, qw = invQuat(t1.rotation)
    t.rotation.x = qx
    t.rotation.y = qy
    t.rotation.z = qz
    t.rotation.w = qw
    
    return t


def invQuat(q1):
    d = q1.x*q1.x + q1.y*q1.y + q1.z*q1.z + q1.w*q1.w
    d += 1e-12   # Add epsilon value to avoid div 0 error
    return q1.x/d, -q1.y/d, -q1.z/d, -q1.w/d


def mulQuat(q1, q2):
    x_ = q1.x * q2.w + q1.w * q2.x + q1.y * q2.z - q1.z * q2.y
    y_ = q1.y * q2.w + q1.w * q2.y + q1.z * q2.x - q1.x * q2.z
    z_ = q1.z * q2.w + q1.w * q2.z + q1.x * q2.y - q1.y * q2.x
    w_ = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
    return x_, y_, z_, w_


def mat6diag(v):
    return [(float(v) if i % 7 == 0 else 0.0) for i in range(36)]

                
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