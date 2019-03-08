from Reward import *
from PID import *
import time 
import numpy as np
from simfunctions import *


# forklift control logic
def clip(value):
    if value > 1:
        value = 1
    elif value < -1:
        value = -1
    return value
class SimpleController(Reward):
    pid_control = PID(3,0.03,0.01,2)
    def calculateMotorControl(self, goal):
        des_angle = np.arctan(goal[1]/goal[0]) 
        if goal[0] > 0:
            des_angle -= np.sign(des_angle)*np.pi

        input = clip(self.pid_control.update(des_angle, 0, time.clock())[0])
        print(goal[0][0], goal[1][0], des_angle[0])

        return input, des_angle



# forklift path planner
class ForkliftPlanner():
    seq = 0
    def __init__(self, pallet_pose, dist, y_offset, tolerance=0.2):
        self.tolerance = 0.2
        self.initialize(pallet_pose, dist, y_offset)
    def initialize(self, pallet_pose, dist, y_offset):
        self.seq = 0
        self.calculatePalletTransform(pallet_pose)
        self.getWaypoints(dist, y_offset)
    
    def getForkliftGlobalPose(self, global_pose):
        x,y = forkTransform(global_pose)
        self.global_position = np.asarray(([x], [y], [1]))
        quaternion = [global_pose.orientation.w_val, global_pose.orientation.x_val, global_pose.orientation.y_val, global_pose.orientation.z_val]
        self.global_orientation = convertToEuler(quaternion)


    def calculateForkliftTransform(self):
        self.transform = np.matmul(np.array(([np.cos(self.global_orientation), 
                                                      -np.sin(self.global_orientation),0], [np.sin(self.global_orientation), 
                                                       np.cos(self.global_orientation),0],[0,0,1])), np.array(([1,0,-self.global_position[0][0]], 
                                                        [0,1,-self.global_position[1][0]],[0,0,1])))
    def goalToForkliftFrame(self):
        self.local_goal = np.matmul(self.transform , self.global_goal)
        return self.local_goal
    def calculatePalletTransform(self, pallet):
        pallet_pose = [pallet.position.x_val, pallet.position.y_val]
        quaternion = [pallet.orientation.w_val, pallet.orientation.x_val, pallet.orientation.y_val, pallet.orientation.z_val]
        self.global_pallet_orientation = convertToEuler(quaternion)
        self.pallet_to_global_transform = np.matmul(
                                            np.array(
                                                (
                                                    [1,0,pallet_pose[0]], 
                                                    [0,1,pallet_pose[1]],
                                                    [0,0,1]
                                                    ) ), 
                     
                                 np.array(
                                     (
                                         [np.cos(self.global_pallet_orientation), -np.sin(self.global_pallet_orientation),0], 
                                         [np.sin(self.global_pallet_orientation), np.cos(self.global_pallet_orientation),0],
                                         [0,0,1]
                                      )     )   )
        self.global_to_pallet_transform = np.matmul(
                                np.array(
                                     (
                                         [np.cos(-self.global_pallet_orientation), -np.sin(-self.global_pallet_orientation),0], 
                                         [np.sin(-self.global_pallet_orientation), np.cos(-self.global_pallet_orientation),0],
                                         [0,0,1]
                                      )     ),
                                            np.array(
                                                (
                                                    [1,0,-pallet_pose[0]], 
                                                    [0,1,-pallet_pose[1]],
                                                    [0,0,1]
                                                    ) )
                     
   )
    def getWaypoints(self, distance_from_pallet, y_offset):
        x = np.array((-1))
        y = np.array((0))

        homogenous_array = np.concatenate((x.reshape((1,-1)),y.reshape((1,-1)), np.ones((1,x.size))), axis=0)
        self.global_path = np.matmul(self.pallet_to_global_transform, homogenous_array)
        self.global_goal = self.global_path[:,0]

    def setGoal(self):
        forklift_in_goal_frame = self.global_to_pallet(self.global_position)
        self.distance = forklift_in_goal_frame[0][0]
        goal_point = np.asarray(([self.distance - 0.7], [0], [1] ))
        self.global_goal = self.pallet_to_global(goal_point)


    def global_to_forklift(self, vector):
        return np.matmul(self.transform, vector)

    def global_to_pallet(self, vector):
        return np.matmul(self.global_to_pallet_transform, vector)

    def pallet_to_global(self, vector):
        return np.matmul(self.pallet_to_global_transform, vector)

    def update(self, forklift_pose):
        self.getForkliftGlobalPose(forklift_pose)
        self.calculateForkliftTransform()
        self.setGoal()
        return self.global_goal, self.distance
        
        



        
