from ForkliftDynamics import ForkliftDynamics
import rospy
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped, TwistStamped, Twist, Point, Point32

from nav_msgs.msg import Odometry, Path, OccupancyGrid
from std_msgs.msg import Float64

from tf2_geometry_msgs import PointStamped
from tf2_msgs.msg import TFMessage

import tf2_ros
from tf_conversions import transformations

class MapData:
    image = None
    resolution = None
    width = None
    height = None
    origin = np.array([0,0,0])


class MotionPlanner:
    ## Initialization
    def __init__(self, frequency=10):
        self.map = MapData()
        self.time_step = 1/frequency
        self.setup_ros()
        self.trigger = [False, False]
        self.motion_primitives = [[1, 0], [1, 0.5], [-1, 0.5], [-1,0], [-1 -0.5], [1, -0.5]]

    def setup_ros(self):
        self.estimator = ForkliftDynamics()
        self.map_sub = rospy.Subscriber("/costmap", OccupancyGrid, self.map_cb)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.goal_sub = rospy.Subscriber("/goal", PoseStamped, self.goal_cb)

        self.pose_pub = rospy.Publisher("planner/pose", PoseStamped, queue_size=1)
        self.plan_pub = rospy.Publisher("planner/global_path", Path, queue_size=1)

    ## subscriber callbacks
    def odom_cb(self, odom_msg):
        velocity = np.norm([odom_msg.twist.linear.x, odom_msg.twist.linear.y])
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        theta = 2*np.arcsin(odom_msg.pose.pose.orientation.z)
        #self.estimator.setKinematics(velocity, x,y, theta)
    def goal_cb(self, pose_msg):
        #goal_th = 2*np.arcsin(pose_msg.pose.orientation.z)
        self.goal = [pose_msg.pose.position.x, pose_msg.pose.position.y, goal_th]
        self.trigger[0] = True
    def map_cb(self, map_msg):
        self.map.height = map_msg.info.height
        self.map.width = map_msg.info.width
        self.map.resolution = map_msg.info.resolution
        self.map.origin[0] = map_msg.origin.position.x
        self.map.origin[1] = map_msg.origin.position.y
        self.map.origin[2] = map_msg.origin.position.z
        self.occupancy_grid = map_msg.data
        self.trigger[1] = True

    def update(self):
        if all(self.trigger):
            self.plan()
            #self.control()
            self.estimator.estimateKinematics(self.throttle, self.steering, self.time_step)
            #self.control_pub.publish(self.control_msg)
            #self.pose_pub.publish(self.pose_msg)
            self.generatePathMsg()
            self.plan_pub.publish(self.path_msg)

    def plan(self):
        # Plan trajectory
        while len(self.plan_points) < 10: #np.linalg.norm([self.estimator.x - self.goal[0], self.estimator.y-self.goal[1]]) > self.goal_tolerance:
            next_point = self.minCost()
            self.plan_points += next_point[0]
            self.estimator.estimateKinematics(next_point[1][0], next_point[1][1], self.time_step)
    
    def minCost(self):
        cost = np.inf
        for controls in self.motion_primitives:
            x,y, = self.estimator.extrapolate(controls[0], controls[1], self.time_step)
            move_cost = np.norm([self.goal[0] - x, self.goal[1] - y])
            if move_cost < cost:
                next_point = [[x,y], controls]
        return next_point

    #def control(self):

    def generatePathMsg(self):
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'world'
        self.path_msg.header.stamp = rospy.get_rostime()
        for i in range(self.plan_points):
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.pose.position.x = self.plan_points[i][0]
            pose.pose.position.y = self.plan_points[i][1]
            self.path_msg.poses += [pose]




