from Reward import *
from PID import *
import time 
import numpy as np
class SimpleController(Reward):
    pid_control = PID(5,0,0)

    def calculateMotorControl(self):
        input = self.y_offset_ #self.pid_control.update(0, -self.y_offset_, time.clock())
        print(self.distance_, self.y_offset_, input)
    #    if self.distance_ > 2:



        
    #def sendMotorControl(self):
    #    return self.calculateMotorControl()




class ForkliftPlanner():
    client = airsim.CarClient()

    def getForkliftGlobalPose(self):
        global_pose = getCar(self.client)
        self.global_position = [global_pose.position.x_val, global_pose.position.y_val]
        quaternion = [global_pose.orientation.w_val, global_pose.orientation.x_val, global_pose.orientation.y_val, global_pose.orientation.z_val]
        self.global_orientation = convertToEuler(quaternion)
    def calculateForkliftTransform(self):
        self.transform = np.matmul(np.array(([np.cos(self.global_orientation), 
                                                      -np.sin(self.global_orientation),0], [np.sin(self.global_orientation), 
                                                       np.cos(self.global_orientation),0],[0,0,1])), np.array(([1,0,-self.global_position[0]], 
                                                        [0,1,-self.global_position[1]],[0,0,1])))
    def goalToForkliftFrame(self):
        self.local_goal = np.matmul(self.transform_matrix_ , np.array(([self.global_goal[0]], [self.global_goal[1]], [1])))
    def calculatePalletTransform(self, pallet):
        pallet_pose = [5,5]
        self.global_pallet_orientation=0
        #pallet_pose = [pallet.pose.position.x_val, pallet_pose.position.y_val]
        #quaternion = [pallet.orientation.w_val, pallet.orientation.x_val, pallet.orientation.y_val, pallet.orientation.z_val]
        #self.global_pallet_orientation = convertToEuler(quaternion)
        self.pallet_to_global_transform = np.matmul(
                                            np.array(
                                                (
                                                    [1,0,pallet_pose[0]], 
                                                    [0,1,pallet_pose[1]],
                                                    [0,0,1]
                                                    ) ), 
                     
                                 np.array(
                                     (
                                         [np.cos(-self.global_pallet_orientation), -np.sin(-self.global_pallet_orientation),0], 
                                         [np.sin(-self.global_pallet_orientation), np.cos(-self.global_pallet_orientation),0],
                                         [0,0,1]
                                      )     )   )
                                         
    def getWaypoints(self, distance_from_pallet, y_offset):
        x = np.arange(distance_from_pallet,-2,0.1)
        y = 1/(1+np.exp(-x-4)) - 1
        if y_offset > 0:
            y = -y
        x = np.append(x, 0)
        y = np.append(y,0)
        homogenous_array = np.concatenate((x.reshape((1,-1)),y.reshape((1,-1)), np.ones((1,x.size))), axis=0)
        self.global_path = np.matmul(self.pallet_to_global_transform, homogenous_array)

    def 
        



        
