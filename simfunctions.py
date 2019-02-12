import airsim
from airsim.types import Vector3r, Quaternionr, Pose
import math
from time import sleep
from argparse import ArgumentParser
import numpy as np
import pptk
import matplotlib.pyplot as plt


def setRandomPose(center, offset_range):
    random_vector = np.random.random_sample((2,1))
    unit_vector = random_vector / np.linalg.norm(random_vector)
    displacement_vector = (offset_range[1]-offset_range[0])*(np.random.random_sample())*2*unit_vector + offset_range[0]
    displacement_vector = displacement_vector.reshape(-1,)
    position = Vector3r(x_val = center[0]+displacement_vector[0], 
                        y_val = center[1] + displacement_vector[1], 
                        z_val = center[2])
    rotation = 2*np.sin(2*np.pi*np.random.random_sample())
    orientation = Quaternionr(z_val=rotation)
    pose = Pose(position_val = position, orientation_val = orientation)
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

def setCar(client):
    client.simSetVehiclePose(Pose(Vector3r(0,0,1), Quaternionr()), True)
def getCar(client):
    pose = client.simGetVehiclePose()
    # xy_location = [pose.position.x_val, pose.position.y_val]
    # z_rotation = pose.orientation.z_val
    # return xy_location, z_rotation
    return pose

def setup():
    cli = airsim.CarClient()
    center = getPallet(cli)
    spot = [5,0,center.position.z_val]
    setPallet(cli,spot,0)
    setCar(cli)

    cli.enableApiControl(False)
    car_controls = airsim.CarControls()
    sleep(0.1)
    return cli, car_controls, getPallet(cli)#spot

def ping(client, view=False):
    l = client.getLidarData()
    cost = getCost(np.asarray(l.point_cloud))

    if view:
        pptk.viewer(np.asarray(l.point_cloud).reshape(-1,3))

    return cost