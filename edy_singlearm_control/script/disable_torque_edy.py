#!/usr/bin/env python

import math
import sys
import rospy
from dynamixel_controllers.srv import *
from dynamixel_msgs.msg import *

# Enable/disable motor torque
def set_torque_enable(joint, enable):
        torque_enable_srv = ''
        torque_enable_srv = "edy_position_controller_l_" + joint + "/torque_enable"
	rospy.wait_for_service(torque_enable_srv)
	try:
		torque_enable_req = rospy.ServiceProxy(torque_enable_srv, TorqueEnable)
		torque_enable_req(enable)
		print( "(Joint: " + joint + ") Torque Enable set to " + str(enable))
	except rospy.ServiceException, e:
		print( "(Joint: " + joint + ") Torque Enable service call failed: %s"%e )

def usage():
    return "%s"%sys.argv[0]

if __name__ == "__main__":

	joint_list = [ "1", "2", "3", "4", "5", "6", "7", "8"]

	rospy.init_node('disable_torque_edy')

	for joint in joint_list:
		# Disable 'torque enable' of current joint/motor
		set_torque_enable(joint, False)

rospy.sleep(0.5)
