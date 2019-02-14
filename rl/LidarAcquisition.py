import airsim
from airsim.types import Vector3r, Quaternionr, Pose
import numpy as np
from time import sleep

class LidarAcquisition:
    client=[]
    return_size=1024/2
    def __init__(self, c):
        self.client = c
    def convertToPolar(self,cloud):
        r = np.sqrt(np.power(cloud[:,0],2) + np.power(cloud[:,1],2))
        theta = np.arctan(cloud[:,1]/cloud[:,0])
        return r, theta

    def getPolar(self,chop=False):
        l = self.client.getLidarData().point_cloud
        shaped_cloud = np.asarray(l).reshape(-1,3)[:,0:2]
        r, theta = self.convertToPolar(shaped_cloud)
        if chop:
            height = len(r)
            r = r[round(height/2)-round(self.return_size/2):round(height/2)+round(self.return_size/2)]
            theta = theta[round(height/2)-round(self.return_size/2):round(height/2)+round(self.return_size/2)]
            return np.concatenate((r,theta))
        else:
            return r, theta
