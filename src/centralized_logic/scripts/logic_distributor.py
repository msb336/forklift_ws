import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from tf2_geometry_msgs import PoseStamped

from std_msgs.msg import Bool, Int8, String
from actionlib_msgs.msg import GoalID
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
    PICKUP          = 1
    ALIGN_PICKUP    = 2
    LOAD            = 3
    DELIVER         = 4
    ALIGN_DELIVERY  = 5
    UNLOAD          = 6
    GO_HOME         = 7
    CHARGE          = 8
    IDLE            = 9
class OPERATION(Enum):
    TRAVERSING      = 1
    ALIGNING        = 2
    MOVING_FORKS    = 3
    IDLE            = 4

class GOAL:
    def __init__(self):
        self.traversed          = False
        self.aligned            = False
        self.loaded             = False
        self.pickup_requested   = False
        self.delivery_complete  = False


class FORKS(Enum):
    DOWN    = 1
    HALFWAY = 2
    UP      = 3
    MOVING  = 4

class ForkliftOperator:
    command     = 0
    calls       = 0
    status      = FORKS.DOWN
    lift_time = 4 
    def lift(self):
        finished = False
        if self.status is FORKS.UP:
            finished = True
        elif self.status is not FORKS.MOVING:
            self.start_time = time.time()
            self.status = FORKS.MOVING
            print("lifting")
        if time.time() - self.start_time < self.lift_time:
                self.command = 1
        else:
            self.command = 0
            self.status = FORKS.UP
            finished = True
            return finished


    def lower(self):
        finished = True
        if self.status is FORKS.HALFWAY:
            finished = False
        elif self.status is not FORKS.MOVING:
            self.start_time = time.time()
            self.status = FORKS.MOVING
            print("lowering")
        if time.time() - self.start_time < self.lift_time/2 + 1:
                self.command = 2
        else:
            self.command = 0
            self.status = FORKS.HALFWAY
            finished = False
        return finished
    def drop(self):
        finished = True
        if self.status is FORKS.DOWN:
            finished = False
        elif self.status is not FORKS.MOVING:
            self.start_time = time.time()
            self.status = FORKS.MOVING
            print("lowering")
        if time.time() - self.start_time < self.lift_time/2 + 1:
            self.command = 2
        else:
            self.command = 0
            self.status = FORKS.DOWN
            finished = False
        return finished






######## LOGIC DISTRIBUTOR CLASS ##########################
class LogicDistributor:
    robot_x             = np.inf
    robot_y             = np.inf
    goal_x              = 0
    goal_y              = 0
    distance_from_goal  = np.inf
    controller          = ""

    control_logic       = TASK.IDLE
    operation_status    = OPERATION.IDLE
    forklift_operator   = ForkliftOperator()
    goal_status         = GOAL()

    def __init__(self):
        self.setup_ros()
   
    def setup_ros(self):

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.delivery_sub = rospy.Subscriber("/pickup", PoseStamped, self.pickup_cb)
        self.pose_sub = rospy.Subscriber("/airsim/pose", PoseStamped, self.pose_cb)
        self.pallet_sub = rospy.Subscriber("/airsim/pallet_pose", PoseStamped, self.pallet_location_cb)

        self.traverse_goal_pub = rospy.Publisher("/airsim/goal", PoseStamped, queue_size=1)
        self.align_goal_pub = rospy.Publisher("/ml/goal", PoseStamped, queue_size=1)

        self.pallet_spawn_pub = rospy.Publisher("/airsim/teleport_pallet", PoseStamped, queue_size = 1)
        self.control_logic_pub = rospy.Publisher("/logic/controller", String, queue_size=1)

        self.forklift_pub = rospy.Publisher("/airsim/forks", Int8, queue_size=1)
        self.move_base_cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size = 10)

        self.setDeliveryGoal()
        self.setHomeGoal()
        #self.setPickupGoal()
        


    def update(self):
        self.checkDistance()
        self.determineGoal()
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
            self.forklift_operator.drop()
            self.charge()
        else:
            self.idle()

        self.control_logic_pub.publish(self.controller)
        self.forklift_pub.publish(self.forklift_operator.command)



######### control logic ####################
    def checkDistance(self):
        self.distance_from_goal = np.linalg.norm(np.array((self.robot_x-self.goal_x, self.robot_y - self.goal_y)))
        if self.operation_status is OPERATION.TRAVERSING and self.distance_from_goal < 2:
            self.goal_status.traversed = True
        if self.operation_status is OPERATION.ALIGNING and self.distance_from_goal < 0.9:

            self.goal_status.aligned = True
    def determineGoal(self):
        if self.goal_status.pickup_requested:
            if self.goal_status.traversed is False:
                self.goal_x = self.pickup_goal_x
                self.goal_y = self.pickup_goal_y
                self.control_logic = TASK.PICKUP
            elif self.goal_status.aligned is False:
                self.goal_x = self.loading_goal_x
                self.goal_y = self.loading_goal_y
                self.control_logic = TASK.ALIGN_PICKUP
            else:
                self.control_logic = TASK.LOAD

        elif self.goal_status.loaded:
            if self.goal_status.traversed is False:
                self.goal_x = self.dropzone_goal_x
                self.goal_y = self.dropzone_goal_y
                self.control_logic = TASK.DELIVER
            elif self.goal_status.aligned is False:
                self.goal_x = self.delivery_goal_x
                self.goal_y = self.delivery_goal_y
                self.control_logic = TASK.ALIGN_DELIVERY
            else:
                self.control_logic = TASK.UNLOAD
        elif self.goal_status.delivery_complete:
            if self.goal_status.traversed is False:
                self.goal_x = self.home_goal_x
                self.goal_y = self.home_goal_y
                self.control_logic = TASK.GO_HOME
            elif self.goal_status.aligned is False:
                self.goal_x = self.charger_goal_x
                self.goal_y = self.charger_goal_y
                self.control_logic = TASK.CHARGE
            else:
                self.control_logic = TASK.IDLE








###### OPERATION COMMANDS ###########################
    def idle(self):
        if self.operation_status is not OPERATION.IDLE:
            self.operation_status = OPERATION.IDLE
            print("idling")
            self.controller = ""
            self.goal_status = GOAL()
    def pickup(self):
        if self.operation_status is not OPERATION.TRAVERSING:
            #self.setPickupGoal()
            print("heading to pickup location")
            self.controller = "traverse"
            self.operation_status = OPERATION.TRAVERSING
            self.goal_status.delivery_complete = False
    def alignPickup(self):
        if self.operation_status is not OPERATION.ALIGNING:
            self.move_base_cancel_pub.publish(GoalID())
            self.align_goal_pub.publish(self.package_pickup_msg)
            print("aligning with package")
            self.controller = "align"
            self.operation_status = OPERATION.ALIGNING

    def load(self):
        self.controller = ""
        self.operation_status = OPERATION.MOVING_FORKS
        ## Raise forks
        self.goal_status.loaded = self.forklift_operator.lift()
        if self.goal_status.loaded:
            self.goal_status.traversed = False
            self.goal_status.aligned = False
            self.goal_status.pickup_requested = False
    def deliver(self):
        if self.operation_status is not OPERATION.TRAVERSING:
            self.traverse_goal_pub.publish(self.dropzone_msg)
            self.operation_status = OPERATION.TRAVERSING
            print("delivering to drop zone")
            self.controller = "traverse"

    def alignDelivery(self):
        if self.operation_status is not OPERATION.ALIGNING:
            self.align_goal_pub.publish(self.dropoff_exact_msg)
            self.operation_status = OPERATION.ALIGNING
            print("aligning with delivery zone")
            self.controller = "align"

    def unload(self):
        self.controller = ""
        self.operation_status = OPERATION.MOVING_FORKS
        self.goal_status.loaded = self.forklift_operator.lower()
        if not self.goal_status.loaded:
            self.controller = "pullout"
            self.goal_status.delivery_complete = True    
            self.goal_status.traversed = False
            self.goal_status.aligned = False
    def goHome(self):
        if self.operation_status is not OPERATION.TRAVERSING:
            self.traverse_goal_pub.publish(self.home_location_msg)
            self.operation_status = OPERATION.TRAVERSING
            print("heading home")
            
            self.controller = "traverse"

    def charge(self):
        if self.operation_status is not OPERATION.ALIGNING:
            self.align_goal_pub.publish(self.charger_msg)
            self.operation_status = OPERATION.ALIGNING
            print("aligning with charger")
            self.controller = "align"



######## ROS MSG CALLBACKS ##################
    # User goal setting subscriber callback
    def pickup_cb(self,pose_msg):

        self.loading_goal_x = -16.46 #self.pallet_location.pose.position.x 
        self.loading_goal_y = 33.1 #self.pallet_location.pose.position.y
        self.package_pickup_msg = PoseStamped()
        self.package_pickup_msg.header.stamp = rospy.get_rostime()
        self.package_pickup_msg.header.frame_id = "world"
        self.package_pickup_msg.pose.position.x = self.loading_goal_x
        self.package_pickup_msg.pose.position.y = self.loading_goal_y
        self.package_pickup_msg.pose.position.z =  0.178449928761
        self.package_pickup_msg.pose.orientation.z = -0.71325
        self.package_pickup_msg.pose.orientation.x = 0 
        self.package_pickup_msg.pose.orientation.y = 0
        self.package_pickup_msg.pose.orientation.w = 0.707
        world_pose_msg = self.package_pickup_msg

        x = world_pose_msg.pose.position.x
        y = world_pose_msg.pose.position.y
        z = world_pose_msg.pose.position.z

        qw = world_pose_msg.pose.orientation.w
        qz = world_pose_msg.pose.orientation.z
        qx = world_pose_msg.pose.orientation.x
        qy = world_pose_msg.pose.orientation.y
#        self.loading_goal_x = x
#        self.loading_goal_y = y
        self.pickup_goal_x, self.pickup_goal_y = linearExtension(self.loading_goal_x,self.loading_goal_y,[qw,qx,qy,qz], 6)
        goal_theta = world_pose_msg.pose.orientation.z
        
        self.airsim_goal_msg = PoseStamped()
        self.airsim_goal_msg.pose.position.x = self.pickup_goal_x
        self.airsim_goal_msg.pose.position.y = self.pickup_goal_y
        self.airsim_goal_msg.pose.position.z = 0
        self.airsim_goal_msg.pose.orientation.w = world_pose_msg.pose.orientation.w
        self.airsim_goal_msg.pose.orientation.x = world_pose_msg.pose.orientation.x
        self.airsim_goal_msg.pose.orientation.y = world_pose_msg.pose.orientation.y
        self.airsim_goal_msg.pose.orientation.z = world_pose_msg.pose.orientation.z
        self.airsim_goal_msg.header.seq = 1
        self.airsim_goal_msg.header.frame_id = "world"
        self.goal_status.pickup_requested = True

        self.pallet_spawn_pub.publish(self.package_pickup_msg)
        self.traverse_goal_pub.publish(self.airsim_goal_msg)

    # path status subscriber callback
    def path_goal_status_cb(self, status_msg):
        # This will be true when the drone has traversed to either the pickup location, delivery location, or charge location
        self.path_goal_status = status_msg.data

    # ml status subscriber callback
    def ml_goal_status_cb(self, status_msg):
        self.ml_goal_status = status_msg.data

    def pose_cb(self, robot_pose_msg):
        self.robot_x = robot_pose_msg.pose.position.x
        self.robot_y = robot_pose_msg.pose.position.y

    def pallet_location_cb(self, pallet_pose_msg):
        self.pallet_location = pallet_pose_msg


################## MSG CREATORS ######################################

   # def setPickupGoal(self):
   #     self.loading_goal_x = self.pallet_location.pose.position.x#-34.5 
   #     self.loading_goal_y = self.pallet_location.pose.position.y#34
   #     self.package_pickup_msg = self.pallet_location
   #     self.package_pickup_msg.pose.orientation.z = -self.pallet_location.pose.orientation.z
   #     #self.package_pickup_msg = PoseStamped()
   #     #self.package_pickup_msg.pose.position.x = self.loading_goal_x
   #     #self.package_pickup_msg.pose.position.y = self.loading_goal_y
   #     #self.package_pickup_msg.pose.position.z = 0
   #     #self.package_pickup_msg.pose.orientation.w = 0.7
   #     #self.package_pickup_msg.pose.orientation.x = 0
   #     #self.package_pickup_msg.pose.orientation.y = 0
   #     #self.package_pickup_msg.pose.orientation.z =  -0.713 
   #     #self.package_pickup_msg.header.seq = 1
   #     #self.package_pickup_msg.header.frame_id = "world"
    def setDeliveryGoal(self):
        self.delivery_goal_x = -2.3 #-2
        self.delivery_goal_y = 13.7
        self.dropoff_exact_msg = PoseStamped()
        self.dropoff_exact_msg.pose.position.x = self.delivery_goal_x
        self.dropoff_exact_msg.pose.position.y = self.delivery_goal_y
        self.dropoff_exact_msg.pose.position.z = 0
        self.dropoff_exact_msg.pose.orientation.w = dqw = 0.707
        self.dropoff_exact_msg.pose.orientation.x =dqx= 0
        self.dropoff_exact_msg.pose.orientation.y =dqy= 0
        self.dropoff_exact_msg.pose.orientation.z =dqz= -0.707 
        self.dropoff_exact_msg.header.seq = 1
        self.dropoff_exact_msg.header.frame_id = "world"

        self.dropzone_goal_x, self.dropzone_goal_y = linearExtension(self.delivery_goal_x,self.delivery_goal_y,[dqw,dqx,dqy,dqz], 6.5)

        self.dropzone_msg = PoseStamped()
        self.dropzone_msg.pose.position.x = self.dropzone_goal_x
        self.dropzone_msg.pose.position.y = self.dropzone_goal_y
        self.dropzone_msg.pose.position.z = 0
        self.dropzone_msg.pose.orientation.w = dqw = 0.707
        self.dropzone_msg.pose.orientation.x =dqx= 0
        self.dropzone_msg.pose.orientation.y =dqy= 0
        self.dropzone_msg.pose.orientation.z =dqz= -0.707 
        self.dropzone_msg.header.seq = 1
        self.dropzone_msg.header.frame_id = "world"

    def setHomeGoal(self):
        self.charger_goal_x = 11.7
        self.charger_goal_y = -8.5
        self.home_goal_x = 11.7 
        self.home_goal_y = -5.0
        self.home_location_msg = PoseStamped()
        self.home_location_msg.pose.position.x = self.home_goal_x
        self.home_location_msg.pose.position.y = self.home_goal_y
        self.home_location_msg.pose.position.z = 0
        self.home_location_msg.pose.orientation.w = 0.707 
        self.home_location_msg.pose.orientation.x = 0
        self.home_location_msg.pose.orientation.y = 0
        self.home_location_msg.pose.orientation.z = 0.707
        self.home_location_msg.header.seq = 1
        self.home_location_msg.header.frame_id = "world"
    
        self.charger_msg = PoseStamped()
        self.charger_msg.pose.position.x = self.charger_goal_x
        self.charger_msg.pose.position.y = self.charger_goal_y
        self.charger_msg.pose.position.z = 0
        self.charger_msg.pose.orientation.w = 0.707 
        self.charger_msg.pose.orientation.x = 0
        self.charger_msg.pose.orientation.y = 0
        self.charger_msg.pose.orientation.z = 0.707
        self.charger_msg.header.seq = 1
        self.charger_msg.header.frame_id = "world"
