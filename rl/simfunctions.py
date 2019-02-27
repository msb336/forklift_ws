import setup_path 
import airsim
from airsim.types import Vector3r, Quaternionr, Pose
import math
from time import sleep
from argparse import ArgumentParser
import numpy as np
import pptk
import matplotlib.pyplot as plt

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

def ping(client, view=False):
    l = client.getLidarData()
    cost = getCost(np.asarray(l.point_cloud))

    if view:
        pptk.viewer(np.asarray(l.point_cloud).reshape(-1,3))

    return cost

def graphPolar(r,theta):
    print("size", len(r))
    print(np.amin(r), np.amax(r))
    print(np.amin(theta*180/np.pi), np.amax(theta*180/np.pi))

    plt.plot(theta,r, 'ro')
    plt.axis([-np.pi/2, np.pi/2, 0, 60])
    plt.show()
