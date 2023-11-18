# adapted from: https://github.com/rfzeg/apriltag_robot_pose/blob/master/scripts/robot_pose.py
import numpy as np
from numpy import ndarray
from .tf_transformations import *

# Estimates the absolute pose a robot in a map based on the position of the AprilTag markers in the robot's camera field of view
# Broadcasts the transform of odom w.r.t. map to correct odometry drift
# Author: Roberto Zegers R.
# Date: 2019 July

from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Transform
import numpy as np


CAM_ROBOT_TRANS_X = -0.0
CAM_ROBOT_TRANS_Y = -0.0
CAM_ROBOT_TRANS_Z = -0.025



CAM_ROBOT_ROT_X = [0.5, 0.707, 0.5, 0.0]
CAM_ROBOT_ROT_Y = [-0.5, 0.0, 0.5, -0.707]
CAM_ROBOT_ROT_Z = [0.5, 0.0, -0.5, 0.707]
CAM_ROBOT_ROT_W = [0.5, 0.707, 0.5, 0.0]


ROBOT_CAM_TRANS_X = [0.025,0.0,-0.025,0.0]
ROBOT_CAM_TRANS_Y = [0.0,0.025,0.0,-0.025]
ROBOT_CAM_TRANS_Z = 0.0



ROBOT_CAM_ROT_X = [-0.5, -0.707, 0.5, 0.0]
ROBOT_CAM_ROT_Y = [0.5, 0.0, 0.5, -0.707]
ROBOT_CAM_ROT_Z = [-0.5, 0.0, -0.5, 0.707]
ROBOT_CAM_ROT_W = [0.5, 0.707, -0.5, 0.0]

def quartenion2arr(rotation: Quaternion)-> ndarray:

    return np.array([rotation.x, rotation.y, rotation.z, rotation.w])

def inverseTransform(transform: Transform)-> Transform:


    return translist2trans(invTranslist(transform))

def tag_to_robot(tag_cam: Transform, cam_robot: Transform)-> Transform:

    trans_mat = np.array([cam_robot.translation.x, cam_robot.translation.y, cam_robot.translation.z, 1 ])
    rot_mat = np.matmul(quaternion_matrix(quartenion2arr(cam_robot.rotation)), quaternion_matrix(quartenion2arr(tag_cam.rotation)))

    trans_tag_robot= np.matmul(rot_mat, trans_mat)[0:3] + np.array([tag_cam.translation.x, tag_cam.translation.y, tag_cam.translation.z])

    re = Transform()
    rotation = quaternion_from_matrix(rot_mat)
    re.rotation.x, re.rotation.y, re.rotation.z, re.rotation.w = rotation[0], rotation[1], rotation[2], rotation[3]
    re.translation.x = trans_tag_robot[0]
    re.translation.y = trans_tag_robot[1]
    re.translation.z = trans_tag_robot[2]

    return re

# def tag_to_robot(tag_cam: Transform, cam_robot: Transform)-> Transform:

#     trans_mat_tc = np.array([tag_cam.translation.x, tag_cam.translation.y, tag_cam.translation.z, 1 ]).reshape(4,)
#     trans_mat_cr = np.array([cam_robot.translation.x, cam_robot.translation.y, cam_robot.translation.z, 1 ]).reshape(4,)
#     tag_cam_mat = quaternion_matrix(quartenion2arr(tag_cam.rotation))

#     tag_cam_mat[:,3] = trans_mat_tc
#     cam_robot_mat = quaternion_matrix(quartenion2arr(cam_robot.rotation))
#     cam_robot_mat[:,3] = trans_mat_cr
#     tag_robot = tag_cam_mat@cam_robot_mat
#     rot_mat = np.copy(tag_robot)
#     rot_mat[:, 3] = np.zeros((4,))
#     rot_mat[3,3] = 1
#     trans_tag_robot = tag_robot[0:3,-1]

#     # rot_mat = np.matmul(quaternion_matrix(quartenion2arr(cam_robot.rotation)), quaternion_matrix(quartenion2arr(tag_cam.rotation)))

    

#     # trans_tag_robot= np.matmul(rot_mat, trans_mat)[0:3] + np.array([tag_cam.translation.x, tag_cam.translation.y, tag_cam.translation.z])



#     re = Transform()
#     rotation = quaternion_from_matrix(rot_mat)
#     re.rotation.x, re.rotation.y, re.rotation.z, re.rotation.w = rotation[0], rotation[1], rotation[2], rotation[3]
#     re.translation.x = trans_tag_robot[0]
#     re.translation.y = trans_tag_robot[1]
#     re.translation.z = trans_tag_robot[2]

#     return re




def pose2poselist(pose):
    ''' Transforms a pose object into the form of a python list'''
    return [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

def trans2translist(transform: Transform):
    ''' Transforms a Transform object into the form of a python list'''
    return [transform.translation.x, transform.translation.y, transform.translation.z, transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]

def translist2trans(trans_list)->Transform:
    """Transform a Transform list to Transform object"""
    res = Transform()

    res.translation.x, res.translation.y, res.translation.z = trans_list[0], trans_list[1], trans_list[2]
    res.rotation.x, res.rotation.y, res.rotation.z, res.rotation.w = trans_list[3], trans_list[4], trans_list[5], trans_list[6]
    return res

def xyzquat_from_matrix(matrix):
    return list(translation_from_matrix(matrix)) + list(quaternion_from_matrix(matrix))

def matrix_from_xyzquat(arg1, arg2=None):
    return matrix_from_xyzquat_np_array(arg1, arg2).tolist()

def matrix_from_xyzquat_np_array(arg1, arg2=None):

    if arg2 is not None:
        translate = arg1
        quaternion = arg2
    else:
        translate = arg1[0:3]
        quaternion = arg1[3:7]

    return np.dot(compose_matrix(translate=translate),quaternion_matrix(quaternion))

def invPoselist(pose):

    return xyzquat_from_matrix(np.linalg.inv(matrix_from_xyzquat(pose2poselist(pose))))

def invTranslist(trans : Transform):

    return xyzquat_from_matrix(np.linalg.inv(matrix_from_xyzquat(trans2translist(trans))))


def convert_translation_rotation_to_pose(translation, rotation):
        """ Convert from representation of a pose as translation and rotation (Quaternion) tuples to a geometry_msgs/Pose message """
        return Pose(position=Point(x=translation.x,y=translation.y,z=translation.z), orientation=Quaternion(x=rotation.x,y=rotation.y,z=rotation.z,w=rotation.w))

def get_cam_robot_transform(cam_id: int):

    return Transform(translation = Vector3(x = CAM_ROBOT_TRANS_X, y = CAM_ROBOT_TRANS_Y, z = CAM_ROBOT_TRANS_Z), \
        rotation = Quaternion(x = float(CAM_ROBOT_ROT_X[cam_id]), y=float(CAM_ROBOT_ROT_Y[cam_id]), z= float(CAM_ROBOT_ROT_Z[cam_id]), w = float(CAM_ROBOT_ROT_W[cam_id])))

def get_robot_cam_transform(cam_id: int):

    return Transform(translation = Vector3(x = float(ROBOT_CAM_TRANS_X[cam_id]), y = float(ROBOT_CAM_TRANS_Y[cam_id]), z = float(ROBOT_CAM_TRANS_Z)), \
        rotation = Quaternion(x = float(ROBOT_CAM_ROT_X[cam_id]), y=float(ROBOT_CAM_ROT_Y[cam_id]), z= float(ROBOT_CAM_ROT_Z[cam_id]), w = float(ROBOT_CAM_ROT_W[cam_id])))


