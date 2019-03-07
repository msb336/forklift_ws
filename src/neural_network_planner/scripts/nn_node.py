#!/usr/bin/env python
from neural_network_controller import NeuralNetworkController
import rospy

rospy.init_node('nn_node', anonymous=False)
loop_rate = 5 
rate = rospy.Rate(loop_rate)
neural_network_controller = NeuralNetworkController()

while not rospy.is_shutdown():
    neural_network_controller.update()
    rate.sleep()

