import setup_path 
import airsim
from airsim.types import Vector3r, Quaternionr, Pose
import numpy as np
from time import sleep
import argparse
import math



"""
The following euler conversion functions are from https://github.com/matthew-brett/transforms3d
which adapted it from transformations.py, it is needed here until transforms3d is available
as a dependency.

They are for internal use only.
"""


def polynomial(value, function):
    f = 0
    for i in range(len(function)):
        factor = i
        f += function[factor]*np.power(value, factor)
    return f

# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# TODO(lucasw) if sxyz works then eliminate the other possibilities
# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

def _euler_from_matrix(matrix, axes='sxyz'):
    """temporaray import from https://github.com/matthew-brett/transforms3d/blob/master/transforms3d/_gohlketransforms.py for internal use only"""
    firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]


    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j],  M[i, k])
            ay = math.atan2( sy,       M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2( M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az

def _quaternion_matrix(quaternion):
    """temporaray import from https://github.com/matthew-brett/transforms3d/blob/master/transforms3d/_gohlketransforms.py for internal use only"""
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])

def _euler_from_quaternion(quaternion, axes='sxyz'):
    """temporaray import from https://github.com/matthew-brett/transforms3d/blob/master/transforms3d/_gohlketransforms.py for internal use only"""
    return _euler_from_matrix(_quaternion_matrix(quaternion), axes)
def convertToEuler(quaternion):
    return _euler_from_quaternion(quaternion, 'sxyz')[2]


def forkTransform(forklift_pose):
    forklift_x = forklift_pose.position.x_val
    forklift_y = forklift_pose.position.y_val
    rotation_angle = convertToEuler([forklift_pose.orientation.w_val, 
                                     forklift_pose.orientation.x_val, 
                                     forklift_pose.orientation.y_val, 
                                     forklift_pose.orientation.z_val])
    return forklift_x - 0.1*np.cos(rotation_angle), forklift_y -0.5*np.sin(rotation_angle)

class Reward:
    center_ =[0,0]
    rotation_ = 0
    eigen_vector_ = [0,0]
    y_offset_ = 0
    angular_offset_ = 0
    distance_=10000
    transform_matrix_ = np.zeros((3,3))

    def __init__(self, object_pose):
        self.reset(object_pose)

    def reset(self, object_pose):
        self.center_ = [object_pose.position.x_val, object_pose.position.y_val]
        quaternion = [object_pose.orientation.w_val, object_pose.orientation.x_val, object_pose.orientation.y_val, object_pose.orientation.z_val]
        self.rotation_ = convertToEuler(quaternion)
        self.calculateTransformToObjectFrame()



    def calculateTransformToObjectFrame(self):
        self.transform_matrix_ = np.matmul(np.array(([np.cos(self.rotation_), 
                                                      -np.sin(self.rotation_),0], [np.sin(self.rotation_), 
                                                       np.cos(self.rotation_),0],[0,0,1])), np.array(([1,0,-self.center_[0]], 
                                                        [0,1,-self.center_[1]],[0,0,1])))
    def getOffset(self, vehicle_pose):
        
        vehicle_x, vehicle_y = forkTransform(vehicle_pose)
        pose_in_object_frame = np.matmul(self.transform_matrix_ , np.array(([vehicle_x], [vehicle_y], [1])))
        self.y_offset_ = pose_in_object_frame[1][0]
        self.distance_ = pose_in_object_frame[0][0]

        vehicle_rotation = _euler_from_quaternion([vehicle_pose.orientation.w_val, vehicle_pose.orientation.x_val, vehicle_pose.orientation.y_val, vehicle_pose.orientation.z_val])[2]
        vehicle_rotation = vehicle_rotation - np.sign(vehicle_rotation)*np.pi
        del_rot = min(abs(self.rotation_-vehicle_rotation), abs(-vehicle_rotation - self.rotation_), abs(vehicle_rotation-self.rotation_))

        if self.distance_ > 0:
            self.angular_offset_ = del_rot - np.pi
        else:
            self.angular_offset_ = del_rot
        return [self.y_offset_, self.angular_offset_, self.distance_]

    def getReward(self):
        beta = [0,0,5]
        offset_reward = np.exp(-abs(self.y_offset_))
        angle_reward = -polynomial(self.angular_offset_, beta)
        reward = offset_reward #+ angle_reward
        return reward




