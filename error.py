import airsim
from airsim.types import Vector3r, Quaternionr, Pose
import numpy as np

class Reward:
    bounds=[]
    distance=[]
    inlier_error=[]
    outlier_error=[]
    reward=[]
    client=[]
    alpha=[]
    beta=[]
    outlier_multiplier=[]
    def __init__(self, c,alpha=5, beta=2,D=0.1):
        self.client = c
        self.getClosestPoint()
        self.alpha=alpha
        self.beta=beta
        self.D=D
    def convertToPolar(self,cloud):
        r = np.sqrt(np.power(cloud[:,0],2) + np.power(cloud[:,1],2))
        theta = np.arctan(cloud[:,1]/cloud[:,0])
        return r, theta
    def isolate(self,theta):
        target_theta = theta[np.logical_and(theta>=self.bounds[0], theta<=self.bounds[1])]
        target_r = r[np.logical_and(theta>=self.bounds[0], theta<=self.bounds[1])]
        outer_theta = theta[np.logical_or(theta<self.bounds[0], theta > self.bounds[1])]
        outer_r = r[np.logical_or(theta<self.bounds[0], theta > self.bounds[1])]
        return [target_theta, target_r], [outer_theta, outer_r]
    def getError(self):
        r,theta = self.getPolar()
        target,outer = self.isolate(r,theta,self.bounds)
        target_theta = target[0]
        target_r = target[1]
        outer_theta = outer[0]
        outer_r = outer[1]
        self.inlier_error = np.linalg.norm(target_r-self.distance/np.cos(target_theta))
        self.outlier_error = np.linalg.norm(outer_r - self.distance/np.cos(outer_theta))
        return self.inlier_error, self.outlier_error

    def getPolar(self):
        l = self.client.getLidarData().point_cloud
        shaped_cloud = np.asarray(l).reshape(-1,3)[:,0:2]
        r, theta = self.convertToPolar(shaped_cloud)
        return r, theta

    def getClosestPoint(self):
        l = self.client.getLidarData()
        cloud = np.asarray(l.point_cloud).reshape(-1,3)[:,0:2]
        r,theta = self.convertToPolar(cloud)
        self.distance = min(r)
        cutoff = 1.2*self.distance
        lower_theta_bound = min(theta[r <= cutoff] )
        upper_theta_bound = max(theta[r <= cutoff] )
        self.bounds = [lower_theta_bound, upper_theta_bound]

    def calculateReward(self):
        self.reward = self.alpha*np.exp(self.beta/self.inlier_error) + self.D*self.outlier_error -0.5
        return self.reward
    def avgError(self, iterations):
        avgin=0
        avgout=0
        for i in range(iterations):
            r,theta = self.getPolar()
            iner,outer = self.getError(r,theta)
            avgin+=iner
            avgout+=outer
        avgin=avgin/iterations
        avgout=avgout/iterations
        return avgin,avgout
