from staterecording import StateRecording
import roslib; roslib.load_manifest('peg_in_hole') 
import rospy
import ast


def execute_forward(StateAPI,CmdAPI, recording):
	staterecording = StateRecording(StateAPI.get_rarm_state().joint_names)
	print type(recording)
	recording = ast.literal_eval(recording)
	runAllStates(StateAPI,CmdAPI,staterecording.get_names(),recording)

def execute_backward(StateAPI,CmdAPI,recording):
	staterecording = StateRecording(StateAPI.get_rarm_state().joint_names)
	recording = ast.literal_eval(recording)
	runAllStates(StateAPI,CmdAPI,staterecording.get_names(),recording[::-1])


def runAllStates(StateAPI,CmdAPI,names,states):
	print "Executing recorded states"
	for state in states:
		print type(state)
		run_position(StateAPI, CmdAPI, names, tuple(state))

def run_position(StateAPI,CmdAPI,jointnames,jointpositions):
	#sends move order, then waits for it to actually reach that position.
	#returns 1 if it was successful, -1 if it timed out.
	CmdAPI.rarm_assumeJointPositions(jointnames,jointpositions)
	#return StateAPI.block_rarm_joints(jointpositions)
	


	
	
