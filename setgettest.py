import airsim
from airsim.types import Vector3r, Quaternionr, Pose
import math
from time import sleep
from argparse import ArgumentParser
import numpy as np

def setRandomPose(center, range):
    random_vector = np.random.random_sample((2,1))
    unit_vector = random_vector / np.linalg.norm(random_vector)
    displacement_vector = (range[1]-range[0])*np.random.random_sample()*unit_vector + range[0]
    position = Vector3r(x_val = displacement_vector[0][0]-center.x_val, 
                        y_val = displacement_vector[1][0] - center.y_val, 
                        z_val = center.z_val)
    orientation = Quaternionr(z_val = 4*np.random.random_sample()-2)
    pose = Pose(position_val = position, orientation_val = orientation)
    return pose

client = airsim.CarClient()
client.confirmConnection()


vehicle = client.simGetVehiclePose().position
client.simSetVehiclePose(Pose(Vector3r(0,0,1), Quaternionr()), True)
center = Vector3r(-4,0,1)
print("center location: (", center.x_val, center.y_val, center.z_val, ")")
print("car location: (", vehicle.x_val, vehicle.y_val, vehicle.z_val, ")")
range = [0, 0]
# client.simSetObjectPose("pallet", setRandomPose(center, range))
while True:
    client.simSetObjectPose("pallet", setRandomPose(center, range))
    sleep(0.5)