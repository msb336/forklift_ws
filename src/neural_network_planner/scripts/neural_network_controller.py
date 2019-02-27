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

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TwistStamped, Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point32
import tf2_ros
from tf_conversions import transformations
import math
from std_srvs.srv import Empty, EmptyResponse


class NeuralNetworkController:
    def __init__(self):
        self.setup_ros()
        # initialize your controller
    
    def setup_ros(self):
        # This is where we subscribe to our ROS topics there are other topics you may want to add, like the costmap, or the current plan
        self.odom_enu_sub = rospy.Subscriber('/airsim/odom_enu', Odometry, self.odom_enu_cb)
        self.odom_ned_sub = rospy.Subscriber('/airsim/odom_ned', Odometry, self.odom_ned_cb)

        self.pose_sub = rospy.Subscriber("/airsim/pose", PoseStamped, self.pose_cb) # redundant
        self.mono_cam_img_sub = rospy.Subscriber('/airsim/mono/image_raw', Image, self.mono_cam_cb)
        self.depth_img_sub = rospy.Subscriber("/airsim/depth", Image, self.depth_img_cb)
        self.lidar_sub = rospy.Subscriber('/airsim/lidar', PointCloud2, self.lidar_cb)
        self.vehicle_command_pub = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=1)
        
        

    def update(self):
        # Plan
        speed, steering_angle = self.control()
        # send message to controller
    #    vehicle_command_message = self.setVehicleCommandMessage(speed, steering_angle)
    #    self.vehicle_command_pub.publish(vehicle_command_message)

    # build your controller here
    def control(self):
        speed = []
        steering_angle = []
        return speed, steering_angle
    def setVehicleCommandMessage(self, speed, steering_angle):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = '/world'
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = speed
        return msg

    def odom_enu_cb(self, odom_msg):
        self.odom_enu = odom_msg.pose.pose

    def odom_ned_cb(self, odom_msg):
        self.odom_ned = odom_msg.pose.pose

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

    def mono_cam_cb(self, image_msg):
        self.mono_img = np.fromstring(image_msg.data, np.uint8)
    def depth_img_cb(self, depth_image_msg):
        self.depth_img = np.fromstring(depth_image_msg.data, np.uint8)
    def lidar_cb(self, pointcloud_msg):
        
        self.lidar_pointcloud = pc2.read_points(pointcloud_msg, skip_nans=True, field_names=("x", "y", "z"))
