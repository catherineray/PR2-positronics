#! /usr/bin/env python
import rospy
import roslib; roslib.load_manifest("peg_in_hole")
import state_api
import cmd_apiv2 as cmd_api

def execute(StateAPI,CmdAPI):
	init_all(StateAPI,CmdAPI)	

def init_all(StateAPI,CmdAPI):
	#do initialization
	pass
	
