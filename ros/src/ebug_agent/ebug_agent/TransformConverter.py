
import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped, Transform


## map to tag 
# at_0_t = np.array([[0.110, 0.0, 0.0]])
# at_0_r = np.array([0.5, 0.5, 0.5, 0.5])
# map_at_0 = np.vstack([np.hstack([R.from_quat(at_0_r).as_matrix(), at_0_t.transpose()]),np.array([0,0,0,1])])


CAM_ROBOT_ROT_X = [0.5, 0.707, 0.5, 0.0]
CAM_ROBOT_ROT_Y = [-0.5, 0.0, 0.5, -0.707]
CAM_ROBOT_ROT_Z = [0.5, 0.0, -0.5, 0.707]
CAM_ROBOT_ROT_W = [0.5, 0.707, 0.5, 0.0]

"""
This code inverse the cam->tag transform
"""

class TransformConverter(Node):

    def __init__(self):
        super().__init__(self.__class__.__name__)


        self.subscription = self.create_subscription(TFMessage, 'tf_detections', self.listener_callback, 100)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, 'pose', 100)

        self.cameras = []
        for cam_id in range(4):
            cam_robot = Transform()
            
            cam_robot.translation.x = 0.0
            cam_robot.translation.y = 0.0
            cam_robot.translation.z = -0.025

            cam_robot.rotation.x = CAM_ROBOT_ROT_X[cam_id]
            cam_robot.rotation.y = CAM_ROBOT_ROT_Y[cam_id]
            cam_robot.rotation.z = CAM_ROBOT_ROT_Z[cam_id]
            cam_robot.rotation.w = CAM_ROBOT_ROT_W[cam_id]

            self.cameras[cam_id] = cam_robot




    def listener_callback(self, tf_det:TFMessage):
        # Store frame names in variables that will be used to
        # compute transformations
        
        if tf_det.transforms:

            for t in tf_det.transforms:                
                tag_cam = t.transform.inverse()                         # Transform tag to camera
                cam_robot = self.cameras[ int(t.header.frame_id[-1]) ]  # Transform camera to robot
                tag_robot = tag_cam * cam_robot                         # Combined transform, tag to robot


                msg = PoseWithCovarianceStamped()
                msg.header = t.header
                msg.header.frame_id = 'apriltag_'+t.child_frame_id[-1]
                
                msg.pose.pose.position.x = tag_robot.translation.x
                msg.pose.pose.position.y = tag_robot.translation.y
                msg.pose.pose.position.z = tag_robot.translation.z

                msg.pose.pose.orientation.x = tag_robot.rotation.x
                msg.pose.pose.orientation.y = tag_robot.rotation.y
                msg.pose.pose.orientation.z = tag_robot.rotation.z
                msg.pose.pose.orientation.w = tag_robot.rotation.w

                msg.pose.covariance = mat6diag(self.alpha)

                self.publisher.publish(msg)



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