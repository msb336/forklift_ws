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
def isolate(r,theta,bounds):
    target_theta = theta[np.logical_and(theta>=bounds[0], theta<=bounds[1])]
    target_r = r[np.logical_and(theta>=bounds[0], theta<=bounds[1])]
    outer_theta = theta[np.logical_or(theta<bounds[0], theta > bounds[1])]
    outer_r = r[np.logical_or(theta<bounds[0], theta > bounds[1])]

    return [target_theta, target_r], [outer_theta, outer_r]
def getError(r,theta, distance, bounds):
    target,outer = isolate(r,theta,bounds)
    target_theta = target[0]
    target_r = target[1]
    outer_theta = outer[0]
    outer_r = outer[1]

    in_error = np.linalg.norm(target_r-distance/np.cos(target_theta))
    out_error = np.linalg.norm(outer_r - distance/np.cos(outer_theta))
    return in_error, out_error

def setRandomPose(center, offset_range):
    random_vector = np.random.random_sample((2,1))
    unit_vector = random_vector / np.linalg.norm(random_vector)
    displacement_vector = (offset_range[1]-offset_range[0])*(np.random.random_sample()-0.5)*2*unit_vector + offset_range[0]
    displacement_vector = displacement_vector.reshape(-1,)
    position = Vector3r(x_val = center[0]+displacement_vector[0], 
                        y_val = center[1] + displacement_vector[1], 
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
def getLidarCount(client, iterations):
    pointcloud = []
    for i in range(iterations):
        pointcloud += client.getLidarData().point_cloud
    return pointcloud
def getPolar(c):
    # l = getLidarCount(c, 5)
    # print(len(l))
    l = c.getLidarData().point_cloud
    shaped_cloud = np.asarray(l).reshape(-1,3)[:,0:2]
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
def getClosestPoint(client):
    l = client.getLidarData()
    cloud = np.asarray(l.point_cloud).reshape(-1,3)[:,0:2]
    r,theta = convertToPolar(cloud)
    dist = min(r)
    cutoff = 1.2*dist
    lower_theta_bound = min(theta[r <= cutoff] )
    upper_theta_bound = max(theta[r <= cutoff] )

    return dist, [lower_theta_bound, upper_theta_bound]
def setCar(client):
    client.simSetVehiclePose(Pose(Vector3r(0,0,1), Quaternionr()), True)
def setup():
    cli = importThings()
    center = getPallet(cli)
    spot = [5,0,center[2]]
    setPallet(cli,spot,0)
    setCar(cli)
    return cli, spot

def updatePlot(plot, fig, x,y):
    plot.clear()
    plot.plot(x,y, 'ro')
    fig.canvas.draw()

def avgError(c, iterations,dist,bound):
    avgin=0
    avgout=0
    for i in range(iterations):
        r,theta = getPolar(c)
        iner,outer = getError(r,theta,dist,bound)
        avgin+=iner
        avgout+=outer
    avgin=avgin/iterations
    avgout=avgout/iterations
    return avgin,avgout

client, center = setup()
sleep(0.2)

f = open('costanalysis-junk.csv', 'w')
f.write('displacement x,displacement y,rotation,normalized error,inlier cost, outlier cost\n')

tolerance = 5
distance,bounds = getClosestPoint(client)
r,theta = getPolar(client)
target,outer=isolate(r,theta,bounds)
print(distance, bounds)

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(target[0],target[1], 'bo',label='inliers')
ax.plot(outer[0],outer[1],'ro',label='outliers')
ax.axis([-np.pi, np.pi, 0, 20])
fig.show()
inerr, outerr = getError(r,theta,distance,bounds)
input()

max_y = 1
iters = 100
offset_range = [0, max_y]
for i in range(iters+1):
    # y = max_y*(i*2/iters - 1)
    # rotation = 2*np.sin(2*np.pi*i/iters)
    random_pose, displacement, rotation = setRandomPose(center, offset_range)

    rotation_error = min(2-abs(rotation), abs(rotation))
    normalized_error = np.sqrt(np.power(np.linalg.norm(displacement),2) + np.power(rotation_error,2))

    client.simSetObjectPose("pallet", random_pose)
    sleep(0.1)
    inerr,outerr=avgError(client,10,distance,bounds)
    # r,theta = getPolar(client)
    # inerr, outerr = getError(r,theta,distance,bounds)

    f.write("{},{},{},{},{},{}\n".format(displacement[0], displacement[1], rotation_error, normalized_error, inerr, outerr))


    # if len(pcdata) > 3:
    #     cost = getCost(pcdata)
    #     save_data = np.concatenate((displacement.reshape(1,-1), 
    #                                 np.array([rotation_error, error, tolerance,
    #                                  cost]).reshape(1,-1)), axis=1).reshape(-1,)
    #     # np.savetxt(f, save_data.tostring())
    #     for i in range(save_data.size):
    #         f.write("{},".format(save_data[i]))
    #     f.write("\n")

f.close()