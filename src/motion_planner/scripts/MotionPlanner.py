from ForkliftDynamics import ForkliftDynamics
import rospy
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped, TwistStamped, Twist, Point, Point32

from nav_msgs.msg import Odometry, Path, OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
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
    def __init__(self, frequency=1):
        self.show_image = True
        self.map = MapData()
        self.time_step = 1.0/frequency
        self.setup_ros()
        self.trigger = [False, False]
        throttle = 0.5
        self.motion_primitives = [[throttle, 0], [throttle, 0.5], [-throttle, 0.5], [-throttle,0], [-throttle, -0.5], [throttle, -0.5]]
        self.plan_points = []
        self.goal_tolerance = 0.5

    def setup_ros(self):
        self.estimator = ForkliftDynamics()
        self.map_sub = rospy.Subscriber("/costmap", OccupancyGrid, self.map_cb)
        self.map_update_sub = rospy.Subscriber("/costmap_updates", OccupancyGridUpdate, self.map_update_cb)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.goal_sub = rospy.Subscriber("/goal", PoseStamped, self.goal_cb)

        self.pose_pub = rospy.Publisher("planner/pose", PoseStamped, queue_size=1)
        self.plan_pub = rospy.Publisher("planner/global_path", Path, queue_size=1)
        self.point_pub = rospy.Publisher("/planner/point", PointStamped, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    ## subscriber callbacks
    def odom_cb(self, odom_msg):
        velocity = np.norm([odom_msg.twist.linear.x, odom_msg.twist.linear.y])
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        theta = 2*np.arcsin(odom_msg.pose.pose.orientation.z)
        #self.estimator.setKinematics(velocity, x,y, theta)
    def goal_cb(self, world_pose_msg):
        pose_msg = self.tf_buffer.transform(world_pose_msg, 'enu')
        goal_th = 2*np.arcsin(pose_msg.pose.orientation.z)
        self.goal = [pose_msg.pose.position.x, pose_msg.pose.position.y, goal_th]
        self.trigger[0] = True
    def map_cb(self, map_msg):
        self.map_msg = map_msg
        self.map.height = map_msg.info.height
        self.map.width = map_msg.info.width
        self.map.resolution = map_msg.info.resolution
        self.map.origin[0] = map_msg.info.origin.position.x
        self.map.origin[1] = map_msg.info.origin.position.y
        self.map.image = np.asarray(map_msg.data, dtype=np.uint8).reshape(self.map.height, self.map.width)
    def map_update_cb(self, update_msg):
        y = update_msg.y
        x = update_msg.x
        height = update_msg.height
        width = update_msg.width
        self.map.image[y:(y+height), x:(x+width)] = np.asarray(update_msg.data, dtype=np.uint8).reshape(height, width)
        self.trigger[1] = True

    def update(self):
        if all(self.trigger):
            self.plan()
            #self.control()
            #self.estimator.estimateKinematics(self.throttle, self.steering, self.time_step)
            #self.control_pub.publish(self.control_msg)
            #self.pose_pub.publish(self.pose_msg)
            self.generatePathMsg()
            self.plan_pub.publish(self.path_msg)

    def plan(self):
        # Plan trajectory
        #while len(self.plan_points) < 100: #np.linalg.norm([self.estimator.x[-1] - self.goal[0], self.estimator.y[-1]-self.goal[1]]) > self.goal_tolerance:
        next_point = self.minCost()
        self.publishPoint(next_point[0])
        self.plan_points += [next_point[0]]
        self.estimator.estimateKinematics(next_point[1][0], next_point[1][1], self.time_step)
    
    def minCost(self):
        cost = np.inf
        for controls in self.motion_primitives:
            x,y = self.estimator.extrapolate(controls[0], controls[1], self.time_step)
            distance = np.linalg.norm([self.goal[0] - x, self.goal[1] - y])
            move_cost = self.costmapCost(x,y)[0] + distance
            #print(controls, x,y, distance, self.costmapCost(x,y))
            if move_cost < cost:
                self.x_index = self.costmapCost(x,y)[1]
                self.y_index = self.costmapCost(x,y)[2]
                next_point = [[x,y], controls]
                control_input = controls
                cost = move_cost
        print(self.costmapCost(x,y)[0])
        return next_point
    def costmapCost(self, x, y):
        x_index = int(round((x - self.map.origin[1]) / self.map.resolution))
        y_index = int(round((y - self.map.origin[0]) / self.map.resolution))

        return self.map.image[y_index, x_index], x_index, y_index #, y_index]

    #def control(self):

    def generatePathMsg(self):
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'enu'
        self.path_msg.header.stamp = rospy.get_rostime()
        for i in range(len(self.plan_points)):
            pose = PoseStamped()
            pose.header.frame_id = 'enu'
            pose.pose.position.x = self.plan_points[i][0]
            pose.pose.position.y = self.plan_points[i][1]
            self.path_msg.poses += [pose]

    def publishPoint(self, point):
        point_msg = PointStamped()
        point_msg.header.frame_id = 'enu'
        point_msg.header.stamp = rospy.get_rostime()
        point_msg.point.x = point[0]
        point_msg.point.y = point[1]
        point_msg.point.z = 0
        self.point_pub.publish(point_msg)




