"""Brainstorm 
if callback msg string contains "goal" translate to pr2_controllers_msg
send to move_arm

if callback msg string contains "grasp" or "release" translate to pr2_gripper_action msg
send to gripper

def return_current_pose_as_list(cm):
    (pos, rot) = cm.return_cartesian_pose()        
    return pos+rot
"""

def translateGoal(goalX)
	if goalX in ['goalA']
	"""goalA is the start_pose, this is called 3 times by callback"""
		#send to move_arm_, store goal in Array so only one translation?
			
	elif goalX in ['goalB']
	"""goalB is the last recorded pose, this is called once by callback"""

	elif goalX in ['goalC']
	"""goalC moves the arm sinusoidally to find the hole after table contact"""

	elif goalX in ['goalD']
	"""goalD moves the arm downwards until table contact is true, if goalB fails"""

	else: 
		ROS_ERROR "Error, unknown goal. Known goals are [goalA-D]."
	

def sendGoalRequest(msg)
	

