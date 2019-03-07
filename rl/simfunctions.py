#import setup_path 
import airsim
from airsim.types import Vector3r, Quaternionr, Pose
import math
from time import sleep
from argparse import ArgumentParser
import numpy as np
#import pptk
#import matplotlib.pyplot as plt


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

def setRandomPose(client, center, offset_range, rotation_range):
    random_vector = np.random.random_sample((2,1))
    unit_vector = random_vector / np.linalg.norm(random_vector)
    displacement_vector = (offset_range[1]-offset_range[0])*(np.random.random_sample())*unit_vector + offset_range[0]
    displacement_vector = displacement_vector.reshape(-1,)
    position = Vector3r(center.position.x_val+displacement_vector[0], 
                        center.position.y_val + displacement_vector[1], 
                        center.position.z_val)

    rotation = (rotation_range[1]-rotation_range[0])*np.random.random_sample() + rotation_range[0] #2*np.sin(2*np.pi*np.random.random_sample())
    orientation = Quaternionr(z_val=rotation)

    pose = Pose(position_val = position, orientation_val = orientation)
    client.simSetObjectPose("pallet", pose, True)

    return pose, displacement_vector, rotation

def setPalletPose(client,x,y,z,q, qw=0):
    position = Vector3r(x, 
                        y, 
                        z)
    orientation = Quaternionr(w_val=q[0], x_val=q[1], y_val=q[2], z_val=q[3])
    client.simSetObjectPose("pallet", Pose(position, orientation))


def linearExtension(x,y, q, distance):
    rotation_angle = convertToEuler([q[0], 
                                     q[1], 
                                     q[2], 
                                     q[3]])
    #print(x,y, distance, rotation_angle)
    return x + distance*np.cos(rotation_angle), y + distance*np.sin(rotation_angle)

def createPose(center, z_rotation):

    pose = Pose(Vector3r(center[0], center[1], center[2]), Quaternionr(z_val = z_rotation))

    return pose

def getLidarCount(client, iterations):
    pointcloud = []
    for i in range(iterations):
        pointcloud += client.getLidarData().point_cloud
    return pointcloud

def setPose(center, z_rotation):
    pose = Pose(Vector3r(center[0], center[1], center[2]), Quaternionr(z_val = z_rotation))
    return pose
def getPallet(client):
    pose=client.simGetObjectPose("pallet")
    # center = [pose.position.x_val, pose.position.y_val, pose.position.z_val]
    return pose

def convertToRaw(pose):
    x = pose.position.x_val
    y = pose.position.y_val
    z = pose.position.z_val
    wz = pose.orientation.z_val
    return x,y,z,wz
def setPallet(client,center, rotation):
    client.simSetObjectPose("pallet", setPose(center, rotation), True)

def setCar(client, pose):
    #z = getCar(client).position.z_val
    client.simSetVehiclePose(pose, True)
def getCar(client):
    pose = client.simGetVehiclePose()
    return pose

def setup():
    cli = airsim.CarClient()
    center = getPallet(cli)
    car_spot = getCar(cli)
    cli.enableApiControl(True)
    car_controls = airsim.CarControls()
       
    sleep(0.1)
    return cli, car_controls, getPallet(cli), car_spot

#def ping(client, view=False):
#    l = client.getLidarData()
#    cost = getCost(np.asarray(l.point_cloud))

#    if view:
#        pptk.viewer(np.asarray(l.point_cloud).reshape(-1,3))

#    return cost

#def graphPolar(r,theta):
#    print("size", len(r))
#    print(np.amin(r), np.amax(r))
#    print(np.amin(theta*180/np.pi), np.amax(theta*180/np.pi))

#    plt.plot(theta,r, 'ro')
#    plt.axis([-np.pi/2, np.pi/2, 0, 60])
#    plt.show()


def interpret_action(action, client):
    car_controls = airsim.CarControls()
    car_speed = client.getCarState().speed
    car_controls.throttle = -0.65
    car_controls.is_manual_gear = True
    car_controls.manual_gear = -1
    if np.abs(action) > 0.001 and np.abs(action) < 0.1:
        action = np.sign(action)*0.1
    car_controls.steering = action
    #if action == 0:
    #    car_controls.steering = 0.25
    #elif action == 1:
    #    car_controls.steering = -0.25
    #else:
    #    car_controls.steering = 0
    return car_controls


def setRange():
    r=0.05
    tr = 0.03 # 5*np.pi/180
    theta_range =[-tr, tr]
    offset_range =[-r, r]
    return theta_range, offset_range
