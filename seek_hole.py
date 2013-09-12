#! /usr/bin/env python
import record

#----global constants
ERR_BOUNDS = 0.06
#----


def execute(StateAPI, CmdAPI):
	print "Seeking hole"
	search_table(StateAPI,CmdAPI)

#----If the desired height has not been reached, the arm moves in a 2d plane pushing down slightly until the desired height is reached
def search_table(StateAPI,CmdAPI):
	#for j in range(0, 20):
	current = StateAPI.get_rarm_pose(CmdAPI)
		#if check_pose(StateAPI,CmdAPI) == 0:
		#try_position(StateAPI,CmdAPI,current.pose.position.x+0.04, current.pose.position.y + 0.01*(-1)**j, -0.01)
		#print "Move along, this is not the position you are looking for."
	try_position(StateAPI,CmdAPI,current.pose.position.x+0.004, current.pose.position.y + 0.001, -0.01)
	CmdAPI.gripper_action(.09, -1)
	#return "stop"

def try_position(StateAPI,CmdAPI,x,y,z):
	xyz = []; xyz.extend([x, y, z])
	CmdAPI.move_arm_r(xyz)

#----Checks if desired pose has been reached, returns 0 if false, None if true.
def check_pose(StateAPI,CmdAPI):
	end = record.calc_last(StateAPI, CmdAPI)
	curr = StateAPI.get_rarm_pose(CmdAPI)
	if abs(curr.pose.position.z - end.pose.position.z) > ERR_BOUNDS:
		return 0
	
		

