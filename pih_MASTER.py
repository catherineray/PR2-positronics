#! /usr/bin/env python
from state_api import State_API
from cmd_apiv2 import Cmd_API
import roslib; roslib.load_manifest("peg_in_hole")
import rospy
import start
import learn
import perform


def execute():
	rospy.init_node("pih_Master")
	myCommandAPI = Cmd_API()
	myStateAPI = State_API()
	"""
	count = 0
	while count<500:
		arm_controller.lock_rarm(myCommandAPI,myStateAPI)
		count +=1
	"""
	#print myStateAPI.get_rarm_pose(myCommandAPI)
	print "-------------STARTING-------------"
	start.execute(myStateAPI, myCommandAPI)
	print "-------------LEARNING-------------"
	#learn.execute(myStateAPI, myCommandAPI)
	print "-------------PERFORMING-------------"
	perform.execute(myStateAPI,myCommandAPI, learn.execute(myStateAPI, myCommandAPI))

if __name__ == '__main__':
	execute()
	print "Fin"
