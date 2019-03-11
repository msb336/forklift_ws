import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock


from std_msgs.msg import Bool, Int8, String

from tf2_msgs.msg import TFMessage
import tf2_ros
from tf_conversions import transformations


import time
from enum import Enum
import sys





# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# TODO(lucasw) if sxyz works then eliminate the other possibilities
# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

def _euler_from_matrix(matrix, axes='sxyz'):
    """temporaray import from https://github.com/matthew-brett/transforms3d/blob/master/transforms3d/_gohlketransforms.py for internal use only"""
    firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]


    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j],  M[i, k])
            ay = math.atan2( sy,       M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2( M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az

def _quaternion_matrix(quaternion):
    """temporaray import from https://github.com/matthew-brett/transforms3d/blob/master/transforms3d/_gohlketransforms.py for internal use only"""
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])

def _euler_from_quaternion(quaternion, axes='sxyz'):
    """temporaray import from https://github.com/matthew-brett/transforms3d/blob/master/transforms3d/_gohlketransforms.py for internal use only"""
    return _euler_from_matrix(_quaternion_matrix(quaternion), axes)
def convertToEuler(quaternion):
    return _euler_from_quaternion(quaternion, 'sxyz')[2]




def linearExtension(x,y, q, distance):
    rotation_angle = convertToEuler([q[0], 
                                     q[1], 
                                     q[2], 
                                     q[3]])
    #print(x,y, distance, rotation_angle)
    return x + distance*np.cos(rotation_angle), y + distance*np.sin(rotation_angle)


######## FORKLIFT OPERATOR AND TASK ENUMS #####################
class TASK(Enum):
    PICKUP = 1
    ALIGN_PICKUP = 2
    LOAD = 3
    DELIVER = 4
    ALIGN_DELIVERY = 5
    UNLOAD = 6
    GO_HOME = 7
    CHARGE = 8
    IDLE = 9
class STATUS(Enum):
    TRAVERSING = 1
    ALIGNING = 2
    MOVING_FORKS = 3
    IDLE = 4
class FORKS(Enum):
    DOWN = 1
    UP = 2
    MOVING = 3

class ForkliftOperator:
    command = 0
    calls = 0
    status = FORKS.DOWN
    def lift(self):
        finished = False
        if self.status is not FORKS.MOVING:
            self.start_time = time.time()
            self.status = FORKS.MOVING
            print("lifting")
        if time.time() - self.start_time < 5:
                self.command = 1
        else:
            self.command = 0
            self.status = FORKS.UP
            finished = True
            return finished
    def lower(self):
        finished = True
        if self.status is not FORKS.MOVING:
            self.start_time = time.time()
            self.status = FORKS.MOVING
            print("lowering")
        if time.time() - self.start_time < 5:
                self.command = 2
        else:
            self.command = 0
            self.status = FORKS.DOWN
            finished = False
        return finished






######## LOGIC DISTRIBUTOR CLASS ##########################
class LogicDistributor:
    pickup_requested = False
    package_delivered = False
    loaded = False
    controller = ""
    control_logic = TASK.IDLE
    operation_status = STATUS.IDLE
    forklift_operator = ForkliftOperator()

    def __init__(self):
        self.setup_ros()
    
    def setup_ros(self):
        self.delivery_sub = rospy.Subscriber("/pickup", PoseStamped, self.pickup_cb)

        self.path_goal_status_sub = rospy.Subscriber("/airsim/goal_status", Bool, self.path_goal_status_cb)
        self.ml_goal_status_sub = rospy.Subscriber("/ml/goal_status", Bool, self.ml_goal_status_cb)

        self.goal_pub = rospy.Publisher("/airsim/goal", PoseStamped, queue_size=1)
        self.pallet_spawn_pub = rospy.Publisher("/airsim/teleport_pallet", PoseStamped, queue_size = 1)
        self.control_logic_pub = rospy.Publisher("/logic/controller", String, queue_size=1)

        self.forklift_pub = rospy.Publisher("/airsim/forks", Int8, queue_size=1)

        self.setDeliveryGoal()
        self.setHomeGoal()


    def update(self):
        if self.control_logic is TASK.PICKUP:
            self.pickup()
        elif self.control_logic is TASK.ALIGN_PICKUP:
            self.alignPickup()
        elif self.control_logic is TASK.LOAD:
            self.load()
        elif self.control_logic is TASK.DELIVER:
            self.deliver()
        elif self.control_logic is TASK.ALIGN_DELIVERY:
            self.alignDelivery()
        elif self.control_logic is TASK.UNLOAD:
            self.unload()
        elif self.control_logic is TASK.GO_HOME:
            self.goHome()
        elif self.control_logic is TASK.CHARGE:
            self.charge()
        self.control_logic_pub.publish(self.controller)








######## ROS MSG CALLBACKS ##################
    # User goal setting subscriber callback
    def pickup_cb(self,world_pose_msg):
#        world_package_msg = self.tf_buffer.transform(package_location_msg, "world")
        self.package_pickup_msg = world_pose_msg
        x = world_pose_msg.pose.position.x
        y = world_pose_msg.pose.position.y
        z = world_pose_msg.pose.position.z
        qw = world_pose_msg.pose.orientation.w
        qz = world_pose_msg.pose.orientation.z
        qx = world_pose_msg.pose.orientation.x
        qy = world_pose_msg.pose.orientation.y

        goal_x, goal_y = linearExtension(x,y,[qw,qx,qy,qz], 4)
        goal_theta = world_pose_msg.pose.orientation.z
        
        self.airsim_goal_msg = PoseStamped()
        self.airsim_goal_msg.pose.position.x = goal_x
        self.airsim_goal_msg.pose.position.y = goal_y
        self.airsim_goal_msg.pose.position.z = 0
        self.airsim_goal_msg.pose.orientation.w = world_pose_msg.pose.orientation.w
        self.airsim_goal_msg.pose.orientation.x = world_pose_msg.pose.orientation.x
        self.airsim_goal_msg.pose.orientation.y = world_pose_msg.pose.orientation.y
        self.airsim_goal_msg.pose.orientation.z = world_pose_msg.pose.orientation.z
        self.airsim_goal_msg.header.seq = 1
        self.airsim_goal_msg.header.frame_id = "world"

        self.control_logic = TASK.PICKUP

    # path status subscriber callback
    def path_goal_status_cb(self, status_msg):
        # This will be true when the drone has traversed to either the pickup location, delivery location, or charge location
        path_goal_status = status_msg.data
        if path_goal_status:
            # The drone has reached it's goal, let's find out which goal
            if self.pickup_requested:
                # it's gotten into a good proximity to the requested pickup zone to start the ml process
                self.control_logic = TASK.ALIGN_PICKUP

            elif self.loaded:
                self.control_logic = TASK.ALIGN_DELIVERY
            else:
                # it's ready to return to start position
                self.control_logic = TASK.CHARGE

    # ml status subscriber callback
    def ml_goal_status_cb(self, status_msg):
        ml_goal_status = status_msg.data
        if ml_goal_status:
            if self.loaded:
                # means we've hit the drop point
                self.control_logic = TASK.UNLOAD
            elif self.pickup_requested:
                self.control_logic = TASK.LOAD
                print("loading")
            else:
                self.control_logic = TASK.GO_HOME



###### OPERATION COMMANDS ###########################
    def pickup(self):
        if self.operation_status is not STATUS.TRAVERSING:
            print("heading to pickup location")
            self.pallet_spawn_pub.publish(self.package_pickup_msg)
            self.goal_pub.publish(self.airsim_goal_msg)
            self.controller = "ackermann"
            self.pickup_requested = True
            self.operation_status = STATUS.TRAVERSING

    def alignPickup(self):
        if self.operation_status is not STATUS.ALIGNING:
            print("aligning with package")
            self.controller = "ml"
            self.goal_pub.publish(self.package_pickup_msg)
            self.operation_status = STATUS.ALIGNING

    def load(self):
        self.controller = ""
        self.operation_status = STATUS.MOVING_FORKS
        ## Raise forks
        self.loaded = self.forklift_operator.lift()
        if self.loaded:
            self.pickup_requested = False
            self.control_logic = TASK.DELIVER

    def deliver(self):
        if self.operation_status is not STATUS.TRAVERSING:
            self.operation_status = STATUS.TRAVERSING
            print("delivering to drop zone")
            self.controller = "ackermann"
            self.goal_pub.publish(self.dropzone_msg)


    def alignDelivery(self):
        if self.operation_status is not STATUS.ALIGNING:
            self.operation_status = STATUS.ALIGNING
            print("aligning with delivery zone")
            self.controller = "ml"
            self.goal_pub.publish(self.dropzone_msg)

    def unload(self):
        self.operation_status = STATUS.MOVING_FORKS
        self.loaded = self.forklift_operator.lower()
        if not self.loaded:
            self.control_logic = TASK.GO_HOME
            self.package_delivered = True

    
    def goHome(self):
        if self.operation_status is not STATUS.TRAVERSING:
            self.operation_status = STATUS.TRAVERSING
            print("heading home")
            self.goal_pub.publish(self.home_location_msg)
            self.controller = "ackermann"

    def charge(self):
        if self.operation_status is not STATUS.ALIGNING:
            self.operation_status = STATUS.ALIGNING
            print("aligning with charger")
            self.home_location.header.frame_id = "world"
            self.goal_pub.publish(self.home_location_msg)





################## MSG CREATORS ######################################

    def setDeliveryGoal(self):
        self.goal_x = 22.0
        self.goal_y = 6.0
        self.dropzone_msg = PoseStamped()
        self.dropzone_msg.pose.position.x = self.goal_x
        self.dropzone_msg.pose.position.y = self.goal_y
        self.dropzone_msg.pose.position.z = 0
        self.dropzone_msg.pose.orientation.w = 1
        self.dropzone_msg.pose.orientation.x = 0
        self.dropzone_msg.pose.orientation.y = 0
        self.dropzone_msg.pose.orientation.z = 0
        self.dropzone_msg.header.seq = 1
        self.dropzone_msg.header.frame_id = "enu"
    def setHomeGoal(self):
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.home_location_msg = PoseStamped()
        self.home_location_msg.pose.position.x = self.goal_x
        self.home_location_msg.pose.position.y = self.goal_y
        self.home_location_msg.pose.position.z = 0
        self.home_location_msg.pose.orientation.w = 1
        self.home_location_msg.pose.orientation.x = 0
        self.home_location_msg.pose.orientation.y = 0
        self.home_location_msg.pose.orientation.z = 0
        self.home_location_msg.header.seq = 1
        self.home_location_msg.header.frame_id = "enu"
        self.goal_pub.publish(self.home_location_msg)
