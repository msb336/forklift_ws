import airsim
from airsim.types import Vector3r, Quaternionr, Pose
import math
from time import sleep
from argparse import ArgumentParser
import numpy as np
import pptk
import matplotlib.pyplot as plt

def convertToPolar(cloud):
    r = np.sqrt(np.power(cloud[:,0],2) + np.power(cloud[:,1],2))
    theta = np.arctan(cloud[:,1]/cloud[:,0])
    return r, theta


def setRandomPose(center, offset_range):
    random_vector = np.random.random_sample((2,1))
    unit_vector = random_vector / np.linalg.norm(random_vector)
    displacement_vector = (offset_range[1]-offset_range[0])*(np.random.random_sample()-0.5)*2*unit_vector + offset_range[0]
    displacement_vector = displacement_vector.reshape(-1,)
    position = Vector3r(x_val = displacement_vector[0]-center[0], 
                        y_val = displacement_vector[1] - center[1], 
                        z_val = center[2])
    rotation = 2*np.sin(2*np.pi*np.random.random_sample())
    orientation = Quaternionr(z_val=rotation)
    pose = Pose(position_val = position, orientation_val = orientation)
    return pose, displacement_vector, rotation

def setPose(center, z_rotation):
    pose = Pose(Vector3r(center[0], center[1], center[2]), Quaternionr(z_val = z_rotation))
    return pose

def getCost(point_cloud):
    tolerance = 5

    shaped_cloud = point_cloud.reshape(-1,3)[:,0:2]
    # remove all points farther than threshold
    norms = np.linalg.norm(shaped_cloud, axis=1)
    norm_index = norms <= tolerance
    shaped_cloud = shaped_cloud[norm_index, :]
    
    # get edge locations
    max_y_index = np.argmax(shaped_cloud[:,1])
    min_y_index = np.argmin(shaped_cloud[:,1])
    e1 = shaped_cloud[max_y_index, :]
    e2 = shaped_cloud[min_y_index, :]
    min_x = min(shaped_cloud[:,0])
    standard_deviation = np.std(shaped_cloud[:,0])
    # get delE value (y distance between edges)
    cost = np.power((e1[1] - e2[1]-0.808),2)

    return cost
def getPolar(c):
    l = c.getLidarData()
    shaped_cloud = np.asarray(l.point_cloud).reshape(-1,3)[:,0:2]
    r, theta = convertToPolar(shaped_cloud)
    return r, theta

def graphPolar(c):
    r,theta = getPolar(c)
    plt.plot(theta,r, 'ro')
    plt.axis([-np.pi, np.pi, 0, 20])
    plt.show()


def importThings():
    import airsim
    client = airsim.CarClient()
    return client
def getPallet(client):
    pose=client.simGetObjectPose("pallet")
    center = [pose.position.x_val, pose.position.y_val, pose.position.z_val]
    return center
def setPallet(client,center, rotation):
    client.simSetObjectPose("pallet", setPose(center, rotation), True)

def ping(client, view=False):
    l = client.getLidarData()
    cost = getCost(np.asarray(l.point_cloud))

    if view:
        pptk.viewer(np.asarray(l.point_cloud).reshape(-1,3))

    return cost
def setCar(client):
    client.simSetVehiclePose(Pose(Vector3r(0,0,1), Quaternionr()), True)
def setup():
    cli = importThings()
    center = getPallet(cli)
    spot = [-5,0,center[2]]
    setPallet(cli,spot,0)
    setCar(cli)
    return cli, spot

def updatePlot(plot, fig, x,y):
    plot.clear()
    plot.plot(x,y, 'ro')
    fig.canvas.draw()
# client = airsim.CarClient()
# client.confirmConnection()


# vehicle = client.simGetVehiclePose().position
# client.simSetVehiclePose(Pose(Vector3r(0,0,1), Quaternionr()), True)
# center = Vector3r(-4,0,1)
offset_range = [0, 0.2]

client, center = setup()
# f = open('costanalysis.csv', 'w')
# f.write('dispx,dispy,quatz,normalized-error,tolerance,cost\n')

fig = plt.figure()
ax = fig.add_subplot(111)
plot_data, = ax.plot([],[], 'ro')
ax.axis([-np.pi, np.pi, 0, 20])
fig.show()
for i in range(101):
    tolerance = 5
    random_pose, displacement, rotation = setRandomPose(center, offset_range)
    rotation_error = min(2-abs(rotation), abs(rotation))
    error = abs(displacement[1]) + min(2-abs(rotation), abs(rotation))
    client.simSetObjectPose("pallet", random_pose)

    r,theta = getPolar(client)
    updatePlot(ax,fig,theta,r)
    sleep(0.2)
    # pcdata = np.array((10,10,10))
    # for i in range(5):
    #     lidar = client.getLidarData()
    #     pcdata = np.append(pcdata, np.asarray(lidar.point_cloud))
    #     sleep(0.2)
    # if len(pcdata) > 3:
    #     cost = getCost(pcdata)
    #     save_data = np.concatenate((displacement.reshape(1,-1), 
    #                                 np.array([rotation_error, error, tolerance,
    #                                  cost]).reshape(1,-1)), axis=1).reshape(-1,)
    #     # np.savetxt(f, save_data.tostring())
    #     for i in range(save_data.size):
    #         f.write("{},".format(save_data[i]))
    #     f.write("\n")

# f.close()