import airsim
from airsim.types import Vector3r, Quaternionr, Pose
import numpy as np
from time import sleep

class Reward:
    bounds=[]
    length=0.75
    distance=0
    inlier_error=[]
    outlier_error=[]
    reward=[]
    client=[]
    alpha=[]
    beta=[]
    theta_box=[]
    outlier_multiplier=[]
    min_angle = -10*np.pi/180
    max_angle = 10*np.pi/180
    return_size=512
    def __init__(self, c,return_size=512, alpha=[1/5, -1], beta=[0, 10],D=0):
        self.client = c
        # self.getCenterPoint()
        self.alpha=alpha
        self.beta=beta
        self.D=D
        self.return_size=return_size

    def convertToPolar(self,cloud):
        r = np.sqrt(np.power(cloud[:,0],2) + np.power(cloud[:,1],2))
        theta = np.arctan(cloud[:,1]/cloud[:,0])
        return r, theta
    def isolate(self,r,theta):
        target_theta = theta[np.logical_and(theta>=self.bounds[0], theta<=self.bounds[1])]
        target_r = r[np.logical_and(theta>=self.bounds[0], theta<=self.bounds[1])]
        outer_theta = theta[np.logical_or(theta<self.bounds[0], theta > self.bounds[1])]
        outer_r = r[np.logical_or(theta<self.bounds[0], theta > self.bounds[1])]
        return [target_theta, target_r], [outer_theta, outer_r]
    def getError(self):
        r,theta = self.getPolar()
        self.getCenterPoint(r,theta)
        target, outer = self.isolate(r,theta)
        target_theta = target[0]
        target_r = target[1]
        outer_theta = outer[0]
        outer_r = outer[1]
        self.inlier_error = np.linalg.norm(target_r-self.distance/np.cos(target_theta))
        self.outlier_error = np.linalg.norm(outer_r - self.distance/np.cos(outer_theta))*0.1
        return self.inlier_error, self.outlier_error

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

    def getCenterPoint(self,r,theta):
        self.distance = min(r[abs(theta) == min(abs(theta))])
        lower_theta_bound = np.arcsin(-self.length/(2*self.distance))
        upper_theta_bound = np.arcsin(self.length/(2*self.distance))
        self.bounds = [lower_theta_bound, upper_theta_bound]
        return self.distance, self.bounds

    def calculateReward(self):
        inlier_reward = -self.alpha[0] * self.inlier_error + self.beta[0]
        outlier_reward = self.alpha[1] * np.exp(self.beta[1]/self.outlier_error)
        self.reward = inlier_reward + outlier_reward + self.D
        return self.reward#,inlier_reward,outlier_reward
    def avgError(self, iterations):
        avgin=0
        avgout=0
        for i in range(iterations):
            iner,outer = self.getError()
            avgin+=iner
            avgout+=outer
            sleep(0.05)
        avgin=avgin/iterations
        avgout=avgout/iterations
        self.inlier_error = avgin
        self.outlier_error=avgout
        return avgin,avgout
