#! /usr/bin/env python

def execute(StateAPI,CmdAPI):
	assumeStartGoal(StateAPI, CmdAPI, [0.75, -0.4, 0.1 ])
	print "Start pose assumed."

def assumeStartGoal(StateAPI,CmdAPI,xyz):
	CmdAPI.move_arm_r(xyz)

