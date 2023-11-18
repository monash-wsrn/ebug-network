from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Transform
import numpy as np
from spatialmath import *
from spatialmath.base import q2r
from spatialmath import UnitQuaternion as UQ

# Node(
#     package='tf2_ros',
#     executable='static_transform_publisher',
#     arguments = ['--x', '0.11', '--y', '0.0', '--z', '0', '--yaw', '1.5708', '--pitch', '3.142', '--roll', '1.5708', '--frame-id', 'map', '--child-frame-id', 'apriltag_0']
# ),

# Node(
#     package='tf2_ros',
#     executable='static_transform_publisher',
#     arguments = ['--x', '0.0', '--y', '0.15', '--z', '0', '--yaw', '3.1416', '--pitch', '3.142', '--roll', '1.5708', '--frame-id', 'map', '--child-frame-id', 'apriltag_1']
# ),

# Node(
#     package='tf2_ros',
#     executable='static_transform_publisher',
#     arguments = ['--x', '-0.11', '--y', '0.0', '--z', '0', '--yaw', '-1.5708', '--pitch', '3.142', '--roll', '1.5708', '--frame-id', 'map', '--child-frame-id', 'apriltag_2']
# ),

# Node(
#     package='tf2_ros',
#     executable='static_transform_publisher',
#     arguments = ['--x', '0.0', '--y', '-0.15', '--z', '0', '--yaw', '0', '--pitch', '3.142', '--roll', '1.5708', '--frame-id', 'map', '--child-frame-id', 'apriltag_3']
# ),


CAM_ROBOT_TRANS_X = -0.0
CAM_ROBOT_TRANS_Y = -0.0
CAM_ROBOT_TRANS_Z = -0.025

CAM_ROBOT_YAW = [1.5708, 3.1416, -1.5708, 0.0]
CAM_ROBOT_PITCH= [3.1416, 3.1416, 3.1416, 3.1416]
CAM_ROBOT_ROLL= [1.5708, 1.5708, 1.5708, 1.5708]

CAM_0_ROT = SO3.RPY(CAM_ROBOT_YAW[0], CAM_ROBOT_PITCH[0],CAM_ROBOT_ROLL[0])
CAM_0_TRANS = np.array([CAM_ROBOT_TRANS_X, CAM_ROBOT_TRANS_Y, CAM_ROBOT_TRANS_Z])

CAM_ROBOT_0 = SE3.Rt(CAM_0_ROT, CAM_0_TRANS)

def cam_robot(cam_idx):
    cam_rot = SO3.RPY(CAM_ROBOT_YAW[cam_idx], CAM_ROBOT_PITCH[cam_idx],CAM_ROBOT_ROLL[cam_idx])
    cam_trans = np.array([CAM_ROBOT_TRANS_X, CAM_ROBOT_TRANS_Y, CAM_ROBOT_TRANS_Z])

    cam_robot = SE3.Rt(R=cam_rot, t=cam_trans)

    return cam_robot

def tag_cam(cam_tag: Transform):
    rot_mat = UQ([cam_tag.rotation.w, cam_tag.rotation.x, cam_tag.rotation.y, cam_tag.rotation.z]).SO3()
    trans_mat = np.array([cam_tag.translation.x,cam_tag.translation.y,cam_tag.translation.z,])

    cam_tag = SE3.Rt(rot_mat,trans_mat)

    return cam_tag.inv()

def tag_robot(tag_cam:SE3, cam_robot: SE3) -> Pose:

    tag_robot = tag_cam * cam_robot
    trans = tag_robot.t
    q = UQ(tag_robot).vec


    re =Pose()
    re.position.x, re.position.y, re.position.z = trans[0], trans[1], trans[2]

    re.orientation.w, re.orientation.x, re.orientation.y, re.orientation.z = q[0], q[1], q[2], q[3]

    return re
    



