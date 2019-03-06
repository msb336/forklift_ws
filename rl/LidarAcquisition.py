import airsim
from airsim.types import Vector3r, Quaternionr, Pose
import numpy as np
from time import sleep

class LidarAcquisition:
    client=[]
    return_size=484
    y_range = [-10, 10]
    bin = []

    def __init__(self, c, return_size=484, y_range=[-10,10]):
        self.client = c
        self.return_size=return_size
        self.bin = np.zeros((return_size,))
        for i in range(self.return_size):
            self.bin[i] = (y_range[1] - y_range[0])*i/self.return_size + y_range[0]

    def convertToPolar(self,cloud):
        x = np.sqrt(np.power(cloud[:,0],2) + np.power(cloud[:,1],2))
        y = np.arctan(cloud[:,1]/cloud[:,0])
        return x, y
    def interpolate(self,x,y):
        zero_ratio = 0
        x_bin = np.zeros((self.return_size, ))
        for i in range(self.return_size):
            if np.amin(np.abs(y - self.bin[i])) < 0.01:
                idx = (np.abs(y - self.bin[i])).argmin()
                x_bin[i] = x[idx]
            else:
                x_bin[i] = 0
                zero_ratio += 1
        print(zero_ratio/self.return_size)
        return x_bin
    def chop(self,x,y):
        height = len(x)
        rc = x[round(height/2)-round(self.return_size/2):round(height/2)+round(self.return_size/2)]
        yc = y[round(height/2)-round(self.return_size/2):round(height/2)+round(self.return_size/2)]
        #return np.concatenate((rc,yc), axis=0).reshape(-1,1)
        return rc, yc
    def getPolar(self,bin=False):
        l = self.client.getLidarData().point_cloud
        if len(l) >= self.return_size*3:
            shaped_cloud = np.asarray(l).reshape(-1,3)[:,0:2]
            x, y = self.convertToPolar(shaped_cloud)
            if bin and x.size >= self.return_size:
                x_bin = self.interpolate(x, y)
                return x_bin
            else:
                return self.chop(x,y)
        else:
            return []
    def getLidar(self, bin=False):
        l = self.client.getLidarData().point_cloud
        if len(l) >= self.return_size*3:
            shaped_cloud = np.asarray(l).reshape(-1,3)[:,0:2]
            if bin:
                return self.interpolate(shaped_cloud[:,0], shaped_cloud[:,1])
            else:
                return self.chop(shaped_cloud[:,0], shaped_cloud[:,1])
        else:
            return [], []


