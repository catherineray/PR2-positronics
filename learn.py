import record
import playback_fixed as playback
from staterecording import StateRecording
import rospy
import roslib

def execute(StateAPI, CmdAPI):
	recording = record.execute(StateAPI, CmdAPI)
	print "Reverse, reverse."
	playback.execute_backward(StateAPI, CmdAPI, recording)
	return recording

