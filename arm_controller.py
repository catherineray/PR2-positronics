#! /usr/bin/env python/
import time, copy, threading 
import roslib; roslib.load_manifest('peg_in_hole') 
import rospy 
import actionlib 
import actionlib_msgs.msg 
import arm_navigation_msgs.msg

def lock_rarm(StateAPI, CmdAPI): 
	currentstate = StateAPI.get_rarm_state()
	CmdAPI.rarm_assumeJointPositions(currentstate.joint_names,currentstate.actual.positions)








