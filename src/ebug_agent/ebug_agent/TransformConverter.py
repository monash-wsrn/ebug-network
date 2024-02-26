import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_msgs.msg import TFMessage
import numpy as np
from scipy.spatial.transform import Rotation as R


## map to tag 
at_0_t = np.array([[0.110, 0.0, 0.0]])
at_0_r = np.array([0.5, 0.5, 0.5, 0.5])
map_at_0 = np.vstack([np.hstack([R.from_quat(at_0_r).as_matrix(), at_0_t.transpose()]),np.array([0,0,0,1])])


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

        self.declare_parameter('robot_id', 'default')

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value


        self.subscription = self.create_subscription(
            TFMessage,
            f'{self.robot_id}/tf_detections',
            self.listener_callback,
            100)

        self.timer=self.create_timer(30, self.save_callback)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, f'{self.robot_id}/pose', 100)
        self.cam_tag = np.zeros((1,16))
        self.tag_cam = np.zeros((1,16))
        self.alpha=1e-3

    def save_callback(self):

        # np.savetxt('cam_tag.csv',self.cam_tag,delimiter=',')
        # np.savetxt('tag.csv',self.tag_cam,delimiter=',')
        # print('saved')
        return
    
    def cam_robot(self, cam_id:int):
        ## cam_0 to robot
        cam_t = np.array([[0.0, 0.00, -0.025]])
        cam_r = np.array([CAM_ROBOT_ROT_X[cam_id], CAM_ROBOT_ROT_Y[cam_id] , CAM_ROBOT_ROT_Z[cam_id], CAM_ROBOT_ROT_W[cam_id]])
        cam_robot = np.vstack([np.hstack([R.from_quat(cam_r).as_matrix(), cam_t.transpose()]),np.array([0,0,0,1])])

        return cam_robot




    def listener_callback(self, tf_det:TFMessage):
        # Store frame names in variables that will be used to
        # compute transformations
        
        if tf_det.transforms:

            for t in tf_det.transforms:


                # trasformation matrix of cam->tag
                tx,ty,tz = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
                rx,ry,rz,rw = t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w
                rot = R.from_quat([rx, ry, rz, rw]).as_matrix()
                trans = np.array([[tx,ty,tz]])
                cam_tag = np.hstack([rot, trans.transpose()])
                cam_tag = np.vstack([cam_tag,np.array([0,0,0,1])])

                # transformation matrix of tag->cam
                tag_cam = np.linalg.inv(cam_tag)
                tag_cam = tag_cam / tag_cam[-1,-1]
                
              
                tag_robot = tag_cam @ self.cam_robot(int(t.header.frame_id[-1]))


                rotation = tag_robot[0:3,0:3]
                translation = tag_robot[0:3, -1]
                q = R.from_matrix(rotation).as_quat().astype('float')

                msg = PoseWithCovarianceStamped()
                msg.header = t.header
                msg.header.frame_id = 'apriltag_'+t.child_frame_id[-1]

                msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = translation[0], translation[1], translation[2]
                msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q[0], q[1], q[2], q[3]
                covar = np.diag([self.alpha , self.alpha, self.alpha, self.alpha, self.alpha, self.alpha]).flatten().astype('float')
                

                msg.pose.covariance = covar

                self.publisher.publish(msg)
                

                
def main():
    rclpy.init()
    node = TransformTransformConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()