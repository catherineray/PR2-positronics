#! /usr/bin/env python
from state_api import State_API
from cmd_apiv2 import Cmd_API
import roslib; roslib.load_manifest("pih_testing")
import rospy

def execute():
	rospy.init_node("CoordMover")
	myCommandAPI = Cmd_API()
	myStateAPI = State_API()
	print "Position? "	
	instr = raw_input()
	while(instr!="stop"):
		array = instr.split()
		ints = [ float(x) for x in array]
		myCommandAPI.rarm_assumePose(ints)
		print "Position? "
		instr = raw_input()
	#myCommandAPI.rarm_assumePose([0,.5,0])
	#count = 0
	#while count<500:
	#	arm_controller.lock_rarm(myCommandAPI,myStateAPI)
	#	count +=1
	#start.execute(myStateAPI,myCommandAPI)

if __name__ == '__main__':
	execute()
