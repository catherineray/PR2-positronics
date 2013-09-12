import threading
import arm_controller
import roslib; roslib.load_manifest('peg_in_hole') 
import rospy 
from staterecording import StateRecording

#constants
TIME_BETWEEN_RECORDINGS = 1 #in seconds
END_SAME_NUMBER = 5 #how many must be within tolerance to accept that you're done.

recorded = []
#---------

#global vars
global shouldrun
global staterecording
global endpose
#---------

def arm_test(StateAPI, CmdAPI):
	print "Providing arm lesson: within record"; CmdAPI.move_arm_r([0.75, -0.188, 0.1]); CmdAPI.move_arm_r([0.75, -0.188, 0.0])

def init(StateAPI, CmdAPI):
	joints = StateAPI.get_rarm_state().joint_names
	global staterecording
	staterecording = StateRecording(joints)

def execute(StateAPI, CmdAPI):
	init(StateAPI, CmdAPI)
	#global shouldrun
	#shouldrun = True
	lockingthread = LockingThread(StateAPI, CmdAPI)
	lockingthread.start()
	global staterecording
	print "Starting to Record:"
	#-----debugging, remove when testing on actual robot
	print "Providing arm lesson:"; CmdAPI.move_arm_r([0.75, -0.188, 0.1]); CmdAPI.move_arm_r([0.75, -0.188, 0.0])
	#----/debugging
	while(shouldContinue(StateAPI, CmdAPI)):
		rospy.sleep(TIME_BETWEEN_RECORDINGS)
		currentstate = StateAPI.get_rarm_state()
		staterecording.add_states(currentstate.actual.positions)
	print "/Stopped Recording"	
	#shouldrun = False
	lockingthread.join()
	calc_last(StateAPI, CmdAPI)
	print ("RECORDED: "+ str(staterecording.get_states()))
	return staterecording
	#return str(staterecording.get_states())

def calc_last(StateAPI, CmdAPI):
	global endpose
	global staterecording
	states = staterecording.get_states()	
	latest = states[len(states)-1]	
	#------------
	endpose = CmdAPI.get_rarm_fk(staterecording.get_names(),latest)
	return endpose

def shouldContinue(StateAPI, CmdAPI):
	global staterecording
	#last = StateAPI.get_rarm_state()
	states = staterecording.get_states()
	if len(states)<5:
		return True
	latest = states[len(states)-1]
	return any(not(StateAPI.within_tolerance(latest,states[len(states)-x])) for x in range(2,END_SAME_NUMBER))

class LockingThread(threading.Thread):
	def __init__(self,StateAPI,cmdAPI):
		threading.Thread.__init__(self)
		self.sAPI = StateAPI
		self.cAPI = cmdAPI
	def run(self):
		print "Locking"
		while(shouldContinue(self.sAPI, self.cAPI)):
			arm_controller.lock_rarm(self.sAPI, self.cAPI)

