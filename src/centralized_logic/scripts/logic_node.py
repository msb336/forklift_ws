#!/usr/bin/env python
from logic_distributor import LogicDistributor
import rospy

rospy.init_node('logic_node', anonymous=False)
loop_rate = 10 
rate = rospy.Rate(loop_rate)
logic_distributor = LogicDistributor()

while not rospy.is_shutdown():
    logic_distributor.update()
    rate.sleep()

