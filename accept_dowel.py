#! /usr/bin/env python
import rospy

def execute(StateAPI,CmdAPI):
	print "Give me the dowel."
	CmdAPI.gripper_action(0.09, -1)
	rospy.sleep(3)
	CmdAPI.gripper_action(0.02, -1) #check radius
