from staterecording import StateRecording
import roslib; roslib.load_manifest('peg_in_hole') 
import rospy


def execute_forward(StateAPI,CmdAPI, recording):
	#print "correct~~~~~~~~~~~~~~~"
	names = StateAPI.get_rarm_state().joint_names
	runAllStates(StateAPI,CmdAPI,names,recording.get_states())

def execute_backward(StateAPI,CmdAPI,recording):
	#print "correct~~~~~~~~~~~~~~~"
	names = StateAPI.get_rarm_state().joint_names
	runAllStates(StateAPI,CmdAPI,names,recording.get_states()[::-1])


def runAllStates(StateAPI,CmdAPI,names,states):
	#print "Executing recorded states"
	for state in states:
		#print "State type: "+str(type(state))
		run_position(StateAPI, CmdAPI, names, state)

def run_position(StateAPI,CmdAPI,jointnames,jointpositions):
	#sends move order, then waits for it to actually reach that position.
	#returns 1 if it was successful, -1 if it timed out.
	#print "Name type: "+str(type(jointnames))+" Position type: "+str(type(jointpositions))
	CmdAPI.rarm_assumeJointPositions(jointnames,jointpositions)
	#return StateAPI.block_rarm_joints(jointpositions)
	


	
	
