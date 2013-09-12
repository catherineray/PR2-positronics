#! /usr/bin/env python
import rospy
import roslib; roslib.load_manifest()
import start
from state_api import State_API
from cmd_apiv2 import Cmd_API

def execute():
	rospy.init_node("TestPY")
	myStateAPI = State_API()
	myCmd_API = Cmd_API()
	start.execute(myStateAPI,myCmdAPI)
	print "done"

if __name__ == "__main__":
	execute()	

