#! /usr/bin/env python
from state_api import State_API
from cmd_apiv2 import Cmd_API
import roslib; roslib.load_manifest("peg_in_hole")
import rospy
import start
import learn

def execute():
	rospy.init_node("DiagConsole")
	myCommandAPI = Cmd_API()
	myStateAPI = State_API()
	print "Console initialized."
	print "$: ",
	nextline = raw_input()
	while (nextline)!="exit" :
		if nextline == "pose" :
			pose_command(myCommandAPI,myStateAPI)
		if nextline == "currentpose" :
			print myStateAPI.get_rarm_pose(myCommandAPI)
		print "$: ",
		nextline = raw_input()

def pose_command(cmdAPI,stateAPI):
	print "Pose to assume? X Y Z: ",
	posestr = raw_input()
	posearray = posestr.split()
	newarray = [float(x) for x in posearray]
	cmdAPI.move_arm_r(newarray)

if __name__ == "__main__":
	execute()
