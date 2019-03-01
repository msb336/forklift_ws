import airsim
from airsim.types import Vector3r, Quaternionr, Pose
import numpy as np
from time import sleep

class LidarAcquisition:
    client=[]
    return_size=484
    theta_range = [-10*np.pi/180, 10*np.pi/180]
    bin = []

    def __init__(self, c, return_size=484, theta_range=[-10*np.pi/180, 10*np.pi/180]):
        self.client = c
        self.return_size=return_size
        self.bin = np.zeros((np.power(return_size,2),))
        self.theta_range=theta_range
        for i in range(self.return_size):
            self.bin[i] = (self.theta_range[1] - self.theta_range[0])*i/self.return_size + self.theta_range[0]

    def convertToPolar(self,cloud):
        r = np.sqrt(np.power(cloud[:,0],2) + np.power(cloud[:,1],2))
        theta = np.arctan(cloud[:,1]/cloud[:,0])
        return r, theta
    def interpolate(self,r,theta):
        r_bin = np.zeros((self.return_size, ))
        for i in range(self.return_size):
            idx = (np.abs(theta - self.bin[i])).argmin()
            r_bin[i] = r[idx]
        return r_bin
    def chop(self,r,theta):
        height = len(r)
        rc = r[round(height/2)-round(self.return_size/2):round(height/2)+round(self.return_size/2)]
        thetac = theta[round(height/2)-round(self.return_size/2):round(height/2)+round(self.return_size/2)]
        return np.concatenate((rc,thetac), axis=0).reshape(-1,1)
    def getPolar(self,bin=False):
        l = self.client.getLidarData().point_cloud
        if len(l) >= self.return_size*3:
            shaped_cloud = np.asarray(l).reshape(-1,3)[:,0:2]
            r, theta = self.convertToPolar(shaped_cloud)
            if bin and r.size >= self.return_size:
                r_bin = self.interpolate(r, theta)
                return r_bin
            else:
                return self.chop(r,theta)
        else:
            return []
    def getLidar(self, bin=False):
        l = self.client.getLidarData().point_cloud
        if len(l) >= self.return_size*3:
            shaped_cloud = np.asarray(l).reshape(-1,3)[:,0:2]
            if bin:
                return self.interpolate(shaped_cloud[:,0], shaped_cloud[:,1])
            else:
                return shaped_cloud[:,0], shaped_cloud[:,1]
        else:
            return []


