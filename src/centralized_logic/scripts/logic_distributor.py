import rospy
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
            self.start_time = time.clock()
            self.status = FORKS.MOVING
        if time.clock() - self.start_time < 5:
                self.command = 1

        else:
            self.command = 0
            self.status = FORKS.UP
            finished = True
    def LOWER(self):
        finished = True
        if self.status is not FORKS.MOVING:
            self.start_time = time.clock()
            self.status = FORKS.MOVING
        if time.clock() - self.start_time < 5:
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
        self.delivery_sub = rospy.Subscriber("/pickup", Pose, self.pickup_cb)

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
    def pickup_cb(self, package_location_msg):
        self.package_pickup_msg = package_location_msg
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
            else:
                self.control_logic = TASK.GO_HOME



###### OPERATION COMMANDS ###########################
    def pickup(self):
        if self.operation_status is not STATUS.TRAVERSING:
            print("heading to pickup location")
            self.pallet_spawn_pub.publish(self.package_pickup_msg)
            self.controller = "ackermann"
            self.pickup_requested = True
            self.operation_status = STATUS.TRAVERSING

    def alignPickup(self):
        if self.operation_status is not STATUS.ALIGNING:
            print("aligning with package")
            self.pickup_requested = False
            self.controller = "ml"
            self.goal_pub.publish(self.package_pickup_msg)
            self.operation_status = STATUS.ALIGNING

    def load(self):
        self.operation_status = STATUS.MOVING_FORKS
        print("raising forks")
        ## Raise forks
        self.loaded = self.forklift_operator.lift()
        if self.loaded:
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
        print("unloading package")
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
