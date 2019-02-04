import airsim
from airsim.types import Vector3r, Quaternionr, Pose
import math
from time import sleep
from argparse import ArgumentParser
import numpy as np

def setRandomPose(center, offset_range):
    random_vector = np.random.random_sample((2,1))
    unit_vector = random_vector / np.linalg.norm(random_vector)
    displacement_vector = (offset_range[1]-offset_range[0])*np.random.random_sample()*unit_vector + offset_range[0]
    displacement_vector = displacement_vector.reshape(-1,)
    position = Vector3r(x_val = displacement_vector[0]-center.x_val, 
                        y_val = displacement_vector[1] - center.y_val, 
                        z_val = center.z_val)
    rotation = 4*np.random.random_sample()-2
    orientation = Quaternionr(z_val = 0) #rotation)
    pose = Pose(position_val = position, orientation_val = orientation)
    return pose, displacement_vector, rotation

def setPose(center, z_rotation):
    pose = Pose(Vector3r(center[0], center[1], center[2]), Quaternionr(z_val = z_rotation))
    return pose

def getImportantValues(point_cloud):
    tolerance = 5

    shaped_cloud = point_cloud.reshape(-1,3)[:,0:2]
    # remove all points farther than threshold
    norms = np.linalg.norm(shaped_cloud, axis=1)
    norm_index = norms <= tolerance
    shaped_cloud = shaped_cloud[norm_index, :]
    # get minimum distance from object
    min_x = min(shaped_cloud[:,0])
    min_norm = min(norms)

    # x distance std
    x_standard_dev = np.std(shaped_cloud[:,0])
    # get max angle between nearby points
    
    # get edge locations
    max_y_index = np.argmax(shaped_cloud[:,1])
    min_y_index = np.argmin(shaped_cloud[:,1])
    e1 = shaped_cloud[max_y_index, :]
    e2 = shaped_cloud[min_y_index, :]

    # get delE value (y distance between edges)
    difE = e1[1] - e2[1]

    # get delF (distance from nearest point to nearest edge)
    delF1 = e1[0] - min_x
    delF2 = e2[0] - min_x
    values = np.concatenate((np.array([tolerance, min_norm, min_x]), e1, e2, np.array([difE, delF1, delF2, x_standard_dev])))
    return values

client = airsim.CarClient()
client.confirmConnection()


vehicle = client.simGetVehiclePose().position
client.simSetVehiclePose(Pose(Vector3r(0,0,1), Quaternionr()), True)
center = Vector3r(-4,0,1)
# center = [4,0,1]
offset_range = [0, 1]
# client.simSetObjectPose("pallet", setRandomPose(center, offset_range))
f = open('norotation-data.csv', 'w')
f.write('dispx,dispy,quatz,tolerance,min_norm,min_x,edge1_x,edge1_y,edge2_x,edge2_y,difE,delF1,delF2,std\n')
# displacement = np.array((0,0))
for i in range(101):
    random_pose, displacement, rotation = setRandomPose(center, offset_range)
    # rotation = 4*i/100 - 2
    # random_pose = setPose(center, rotation)
    client.simSetObjectPose("pallet", random_pose)
    pcdata = np.array((10,10,10))
    for i in range(5):
        lidar = client.getLidarData()
        pcdata = np.append(pcdata, np.asarray(lidar.point_cloud))
        sleep(0.01)
    if len(pcdata) > 3:
        data = getImportantValues(pcdata)
        save_data = np.concatenate((displacement.reshape(1,-1), np.array([rotation]).reshape(1,-1), data.reshape(1,-1)), axis=1).reshape(-1,)
        # np.savetxt(f, save_data.tostring())
        for i in range(save_data.size):
            f.write("{},".format(save_data[i]))
        f.write("\n")

f.close()