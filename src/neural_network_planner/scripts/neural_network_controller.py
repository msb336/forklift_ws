from airsim.types import Pose, Vector3r, Quaternionr

import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

from std_msgs.msg import Bool

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

class NeuralNetworkController:
    control_implemented = False
    def __init__(self):
        self.setup_ros()
        # initialize your controller
    
    def setup_ros(self):
        # This is where we subscribe to our ROS topics there are other topics you may want to add, like the costmap, or the current plan
        #self.odom_enu_sub = rospy.Subscriber('/airsim/odom_enu', Odometry, self.odom_enu_cb)
        #self.odom_ned_sub = rospy.Subscriber('/airsim/odom_ned', Odometry, self.odom_ned_cb)

        self.pose_sub = rospy.Subscriber("/airsim/pose", PoseStamped, self.pose_cb) # redundant
        #self.mono_cam_img_sub = rospy.Subscriber('/airsim/mono/image_raw', Image, self.mono_cam_cb)
        #self.depth_img_sub = rospy.Subscriber("/airsim/depth", Image, self.depth_img_cb)
        #self.lidar_sub = rospy.Subscriber('/airsim/lidar', PointCloud2, self.lidar_cb)
        self.vehicle_command_pub = rospy.Publisher('/car_cmd', AckermannDriveStamped, queue_size=1)
        self.vehicle_command_sub = rospy.Subscriber('/airsim/control_handoff', Bool, self.control_cb)
        self.pallet_sub = rospy.Subscriber('/airsim/pallet_pose', PoseStamped, self.pallet_cb)

    def update(self):
        if self.control_implemented == True:
            speed, angle = self.control()
            command_msg = self.setVehicleCommandMessage(speed, angle)
            self.vehicle_command_pub.publish(command_msg)
    def control(self):
        local_goal, global_goal = self.planner.update(self.sim_pose)
        steering_angle = self.controller.calculateMotorControl(local_goal)
        speed = -1
        return speed, steering_angle


    def setVehicleCommandMessage(self, speed, steering_angle):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = '/world'
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = speed
        return msg

    #def odom_enu_cb(self, odom_msg):
    #    self.odom_enu = odom_msg.pose.pose

    #def odom_ned_cb(self, odom_msg):
    #    self.odom_ned = odom_msg.pose.pose

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
    def pallet_cb(self, pallet_pose_msg):
        pos = Vector3r()
        orientation = Quaternionr()
        
        pos.x_val = pallet_pose_msg.pose.position.x
        pos.y_val = pallet_pose_msg.pose.position.y
        pos.z_val = pallet_pose_msg.pose.position.z
        orientation.w_val = pallet_pose_msg.pose.orientation.w
        orientation.x_val = pallet_pose_msg.pose.orientation.x
        orientation.y_val = pallet_pose_msg.pose.orientation.y
        orientation.z_val = pallet_pose_msg.pose.orientation.z
        self.pallet_pose = Pose(pos, orientation)

    def control_cb(self, bool_msg):
        if bool_msg.data == True:
            self.controller = SimpleController(self.pallet_pose)
            [init_offset, angle, init_dist] = self.controller.getOffset(self.sim_pose)
            self.planner = ForkliftPlanner(self.pallet_pose, init_dist, init_offset)
            self.control_implemented = True
