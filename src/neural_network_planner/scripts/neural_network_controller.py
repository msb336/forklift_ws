from airsim.types import Pose, Vector3r, Quaternionr

import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

from std_msgs.msg import Bool, Int8, String

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


def printPose(pose, name = ""):
    print("{} pose:".format(name))
    print("position ({}, {}, {}".format( pose.position.x_val, pose.position.y_val, pose.position.z_val))
    print("orientation ({}, {}, {}, {})".format(pose.orientation.w_val, pose.orientation.x_val, pose.orientation.y_val, pose.orientation.z_val))


class NeuralNetworkController:
    control_implemented = False
    fork_position = False
    aligned = False
    def __init__(self):
        self.setup_ros()
    
    def setup_ros(self):
        # Subscribers
        self.pose_sub = rospy.Subscriber("/airsim/pose", PoseStamped, self.pose_cb) # redundant
        self.vehicle_command_sub = rospy.Subscriber('/logic/controller', String, self.control_cb)
        self.goal_sub = rospy.Subscriber('/airsim/goal', PoseStamped, self.goal_cb)
        
        # Publishers
        self.forklift_pub = rospy.Publisher('/airsim/forks', Int8, queue_size=1)
        self.goal_pose_pub = rospy.Publisher('/ml/goal_pose', PoseStamped, queue_size=10)
        self.local_waypoint_pub = rospy.Publisher('/ml/local_waypoint', PointStamped, queue_size=10)
        self.vehicle_command_pub = rospy.Publisher('/ml_cmd', AckermannDriveStamped, queue_size=1)
        self.status_pub = rospy.Publisher('ml/goal_status', Bool, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def update(self):
        if self.control_implemented == True:
            speed, angle = self.control()
            command_msg = self.setVehicleCommandMessage(speed, angle)
            self.vehicle_command_pub.publish(command_msg)
            self.status_pub.publish(self.status)


    def control(self):
        global_goal, distance_from_goal = self.planner.update(self.sim_pose)
        global_msg = self.setPointMsg(global_goal)
        local_goal_msg = self.tf_buffer.transform(global_msg, 'base_link', rospy.Duration(1.0))

        local_goal = [local_goal_msg.point.x, local_goal_msg.point.y]

        steering_angle, goal_angle = self.controller.calculateMotorControl(local_goal)
        self.publishPoseMsg(goal_angle)
        
        local_waypoint_msg = self.setPointMsg(local_goal, "base_link")
        self.local_waypoint_pub.publish(local_goal_msg)
        if distance_from_goal > 0.5:
            speed = -0.65
        else:
            speed = 0
            self.status = True
        return speed, steering_angle



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

    def goal_cb(self, goal_pose_msg):
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

    def control_cb(self, bool_msg):
        if bool_msg.data == "ml":
            self.controller = SimpleController(self.goal_pose)
            [init_offset, angle, init_dist] = self.controller.getOffset(self.sim_pose)
            self.planner = ForkliftPlanner(self.goal_pose, init_dist, init_offset)
            self.control_implemented = True
            self.status = False
        else:
            self.status = False
            self.control_implemented = False
