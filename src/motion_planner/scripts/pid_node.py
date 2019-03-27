#!/usr/bin/env python
from PIDController import PIDController
import rospy

rospy.init_node('nn_node', anonymous=False)
loop_rate = 15 
rate = rospy.Rate(loop_rate)
pid_controller = PIDController()

while not rospy.is_shutdown():
    pid_controller.update()
    rate.sleep()

