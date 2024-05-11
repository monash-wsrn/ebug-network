
import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Transform

import tf2_py._tf2_py as tf2



# CAM_0 = (0, 0,     0) RPY Radians (FRONT)
# CAM_1 = (0, 0,  PI/2) RPY Radians (RIGHT)
# CAM_2 = (0, 0,    PI) RPY Radians (BACK)
# CAM_3 = (0, 0, 3PI/2) RPY Radians (LEFT)

ROBOT_CAM_ROT_X = [0.0,        0.0,  0.0,         0.0]
ROBOT_CAM_ROT_Y = [0.0,        0.0,  0.0,         0.0]
ROBOT_CAM_ROT_Z = [0.0,  0.7071068,  1.0,   0.7071068]
ROBOT_CAM_ROT_W = [0.0,  0.7071068,  0.0,  -0.7071068]

"""
This code inverse the cam->tag transform
"""

class TransformConverter(Node):

    def __init__(self):
        super().__init__(self.__class__.__name__)

        import inspect
        for name, obj in inspect.getmembers(tf2):
            self.get_logger().warn(f'{name} : {obj}')


        self.subscription = self.create_subscription(TFMessage, 'tf_detections', self.listener_callback, 100)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, 'pose', 100)

        self.cameras = [self.get_camera(i) for i in range(4)]
        self.covariance = mat6diag(1e-1)

    
    def get_camera(self, cam_id):
        t = Transform()

        t.translation.x = 0.0
        t.translation.y = 0.0
        t.translation.z = -0.05

        t.rotation.x = ROBOT_CAM_ROT_X[cam_id]
        t.rotation.y = ROBOT_CAM_ROT_Y[cam_id]
        t.rotation.z = ROBOT_CAM_ROT_Z[cam_id]
        t.rotation.w = ROBOT_CAM_ROT_W[cam_id]
        
        return msg2tf(t)


    def listener_callback(self, tf_det:TFMessage):
        if not tf_det.transforms:
            return
        
        for t in tf_det.transforms:    
            tag_id = int(t.child_frame_id[-1])  # Get the AprilTag number, from '0' format
            cam_id = int(str(t.frame_id)[4:])   # Get the camera number, from 'cam_0' format

            tag_cam = msg2tf(t)                         # (tf2.Transform) Apriltag relative to Camera
            cam_tag = tag_cam.inverse()                 # (tf2.Transform) Camera relative to AprilTag
            robot_cam = self.cameras[ cam_id ]          # (tf2.Transform) Robot relative to Camera
            robot_tag = robot_cam.mult(cam_tag)         # (tf2.Transform) Robot relative to AprilTag
            
            # Pose of the robot within the frame of the detected Apriltag
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = t.header.stamp
            msg.header.frame_id = f'apriltag_{tag_id}'
            
            msg.pose.pose.position.x = robot_tag.getOrigin().getX()
            msg.pose.pose.position.y = robot_tag.getOrigin().getY()
            msg.pose.pose.position.z = robot_tag.getOrigin().getZ()

            msg.pose.pose.orientation.x = robot_tag.getRotation().getAxis().getX()
            msg.pose.pose.orientation.y = robot_tag.getRotation().getAxis().getY()
            msg.pose.pose.orientation.z = robot_tag.getRotation().getAxis().getZ()
            msg.pose.pose.orientation.w = robot_tag.getRotation().getW()

            msg.pose.covariance = self.covariance
            self.publisher.publish(msg)



def msg2tf(msg):
    translation = tf2.Vector3(msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z)
    rotation = tf2.Quaternion(msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w)
    return tf2.Transform(rotation, translation)


# def tf2msg(tf, stamp, frame, child_frame):
#     msg = TransformStamped()
#     msg.header.stamp = stamp
#     msg.header.frame_id  = frame
#     msg.child_frame_id = child_frame

#     msg.transform.translation.x = tf.getOrigin().getX()
#     msg.transform.translation.y = tf.getOrigin().getY()
#     msg.transform.translation.z = tf.getOrigin().getZ()  

#     msg.transform.rotation.x = tf.getRotation().getAxis().getX()
#     msg.transform.rotation.y = tf.getRotation().getAxis().getY()
#     msg.transform.rotation.z = tf.getRotation().getAxis().getZ()
#     msg.transform.rotation.w = tf.getRotation().getW()

#     return msg



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