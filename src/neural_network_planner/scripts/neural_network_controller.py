from airsim.types import Pose, Vector3r, Quaternionr

import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

from std_msgs.msg import Bool, Int8, String, Float64

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TwistStamped, Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point32
import tf2_ros
from tf_conversions import transformations
import math
from std_srvs.srv import Empty, EmptyResponse

import sys
sys.path.append('/home/matt/forklift_ws/rl')
from SimpleController import *
from PID import *
from enum import Enum

def printPose(pose, name = ""):
    print("{} pose:".format(name))
    print("position ({}, {}, {}".format( pose.position.x_val, pose.position.y_val, pose.position.z_val))
    print("orientation ({}, {}, {}, {})".format(pose.orientation.w_val, pose.orientation.x_val, pose.orientation.y_val, pose.orientation.z_val))

class CONTROL(Enum):
    STOP = 0
    TRAVERSE = 1
    ALIGN = 2

class NeuralNetworkController:
    

    def __init__(self):
        self.controller = SimpleController()
        self.speed_controller = PID(1,0, 0.6)
        self.car_speed = 0
        self.setup_ros()
        self.control_implementation = CONTROL.STOP
        self.global_path_goal = PoseStamped() 
    def setup_ros(self):
        # Subscribers
        self.pose_sub = rospy.Subscriber("/airsim/pose", PoseStamped, self.pose_cb) # redundant
        self.speed_sub = rospy.Subscriber("/airsim/speed", Float64, self.speed_cb)
        self.control_type_sub = rospy.Subscriber('/logic/controller', String, self.control_type_cb)
        self.goal_sub = rospy.Subscriber('/ml/goal', PoseStamped, self.goal_cb)
        self.plan_sub = rospy.Subscriber('/move_base/TrajectoryPlannerROS/global_plan', Path, self.plan_cb)

        
        # Publishers
        self.forklift_pub = rospy.Publisher('/airsim/forks', Int8, queue_size=1)
        self.goal_pose_pub = rospy.Publisher('/ml/goal_pose', PoseStamped, queue_size=10)
        self.local_waypoint_pub = rospy.Publisher('/ml/local_waypoint', PointStamped, queue_size=10)
        self.vehicle_command_pub = rospy.Publisher('/ml_cmd', AckermannDriveStamped, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def update(self):
        local_goal, direction, distance = self.getGoals()
        throttle, angle  = self.control(local_goal, distance, direction)
        command_msg = self.setVehicleCommandMessage(throttle*np.sign(direction), angle)
        self.vehicle_command_pub.publish(command_msg)

    def getGoals(self):
        if self.control_implementation is CONTROL.ALIGN:
            global_goal, distance_from_goal = self.alignment_planner.update(self.sim_pose)
            distance_from_goal = np.abs(distance_from_goal)
            global_msg = self.setPointMsg(global_goal)
            local_goal_msg = self.tf_buffer.transform(global_msg, 'base_link', rospy.Duration(1.0))
            local_goal = [local_goal_msg.point.x, local_goal_msg.point.y]
            multiplier = -1
        elif self.control_implementation is CONTROL.TRAVERSE and self.global_path_goal.header.frame_id is not "":
            try:
                local_goal_msg = self.tf_buffer.transform(self.global_path_goal, "base_link", rospy.Duration(1.0))
            except:
                local_goal_msg = self.global_path_goal
                local_goal_msg.pose.position.x = 0
                local_goal_msg.pose.position.y = 0 
            local_goal = [local_goal_msg.pose.position.x, local_goal_msg.pose.position.y]
            distance_from_goal = 5 
            multiplier = 1/0.7
        else:
            local_goal = [0,0]
            multiplier = 0
            distance_from_goal = 0
        return local_goal, multiplier, distance_from_goal

    def control(self, local_goal, distance_from_goal, direction):
        steering_angle, goal_angle = self.controller.calculateMotorControl(local_goal, direction)
        self.publishPoseMsg(goal_angle)
        
        local_waypoint_msg = self.setPointMsg(local_goal, "base_link")
        self.local_waypoint_pub.publish(local_waypoint_msg)
        if distance_from_goal >= 5:
            speed = 2 
        elif distance_from_goal > 4: 
            speed = 0.8#(distance_from_goal-2)/10 + 0.7 if (distance_from_goal-2)/10 > 0 else 0.7 
        elif distance_from_goal > 1:
            speed = 1
        elif distance_from_goal > 0:
            speed = 0.5
            print(distance_from_goal, speed)
        elif distance_from_goal > 0:
            speed = 0.1
        else:
            speed = 0
            self.status = True
        if speed != 0:
            throttle = self.speed_controller.update(speed, np.abs(self.car_speed), time.time()) 

        else:
            throttle = speed
        #print("[controller]: distance {} speed {} throttle {}".format(distance_from_goal, speed, throttle))
        return throttle, steering_angle*direction



    def setVehicleCommandMessage(self, speed, steering_angle):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = 'world'
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = speed
        return msg

    def setPointMsg(self, waypoint, frame='world'):
        msg = PointStamped()
        msg.header.stamp = rospy.get_rostime() #self.transform_time
        msg.header.frame_id = frame
        msg.point.x = waypoint[0]
        msg.point.y = waypoint[1]
        msg.point.z = 0
        return msg
    def publishPoseMsg(self, goal_angle):
        msg = PoseStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = 0 
        msg.pose.position.y = 0
        msg.pose.position.z = 0

        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = np.sin(goal_angle/2)
        msg.pose.orientation.w = np.cos(goal_angle)
        self.goal_pose_pub.publish(msg)

    def pose_cb(self, sim_pose_msg):
        pos = Vector3r()
        orientation = Quaternionr()
        
        pos.x_val = sim_pose_msg.pose.position.x
        pos.y_val = sim_pose_msg.pose.position.y
        pos.z_val = sim_pose_msg.pose.position.z
        orientation.w_val = sim_pose_msg.pose.orientation.w
        orientation.x_val = sim_pose_msg.pose.orientation.x
        orientation.y_val = sim_pose_msg.pose.orientation.y
        orientation.z_val = sim_pose_msg.pose.orientation.z
        self.sim_pose = Pose(pos, orientation)

    def goal_cb(self, goal_pose_msg_enu):
        goal_pose_msg = self.tf_buffer.transform(goal_pose_msg_enu, 'world')
        pos = Vector3r()
        orientation = Quaternionr()
        
        pos.x_val = goal_pose_msg.pose.position.x
        pos.y_val = goal_pose_msg.pose.position.y
        pos.z_val = goal_pose_msg.pose.position.z
        orientation.w_val = goal_pose_msg.pose.orientation.w
        orientation.x_val = goal_pose_msg.pose.orientation.x
        orientation.y_val = goal_pose_msg.pose.orientation.y
        orientation.z_val = goal_pose_msg.pose.orientation.z
        self.goal_pose = Pose(pos, orientation)
        self.controller.reset(self.goal_pose)

    def control_type_cb(self, string_msg):
        self.status = False
        if string_msg.data == "align":
            self.controller.reset(self.goal_pose)
            self.controller.setGains(0.4,0,0.4)
            self.speed_controller = PID(1, 0, 0.2)
            [init_offset, angle, init_dist] = self.controller.getOffset(self.sim_pose)
            self.alignment_planner = ForkliftPlanner(self.goal_pose, init_dist, init_offset)
            self.control_implementation = CONTROL.ALIGN
        elif string_msg.data == "traverse":
            self.control_implementation = CONTROL.TRAVERSE
            self.controller.setGains(0.3,0, 0)
            self.speed_controller = PID(1,0,0.6)
        else:
            self.control_implementation = CONTROL.STOP


    def plan_cb(self, plan_msg):
        try:
            self.global_path_goal = plan_msg.poses[6]
        except:
            self.global_path_goal = plan_msg.poses[0]


    def speed_cb(self, speed_msg):
        self.car_speed = speed_msg.data
