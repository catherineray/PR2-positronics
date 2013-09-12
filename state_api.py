#! /usr/bin/env python
#The python wrapper/API for PR2 running ROS.
import rospy
#from rospy import Time
import roslib; roslib.load_manifest('peg_in_hole') 

#Rin edit - a lot of msg imports
import geometry_msgs.msg
import kinematics_msgs.msg
import kinematics_msgs.srv
import arm_navigation_msgs.msg
import arm_navigation_msgs.srv
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import std_msgs.msg
from sensor_msgs.msg import JointState
from pr2_controllers_msgs.msg import *


#global variables

TOLERANCE = 0.8

	#topics
SUB_RARM_STATE = "/r_arm_controller/state" 
SUB_RGRIPPER_STATE = "/r_gripper_controller/state"
	#--------------


#--------------
class State_API:
	
	def __init__(self):
		self.init_all()
		self.cached_rarm_state = JointTrajectoryControllerState()
		self.cached_rgripper_state = JointControllerState()
		self.armcount = 0
		self.grippercount = 0


	def within_tolerance(self,arrayone,arraytwo):
		return not(any([abs(x-y)>TOLERANCE for x,y in zip(arrayone,arraytwo)]))


#callback methods

	def getArmCount(self):
		return self.armcount

	def getGripperCount(self):
		return self.grippercount


	def rarm_callback(self,msg):
		self.cached_rarm_state = msg
		self.armcount+=1

	def rgripper_callback(self, msg):
		self.cached_rgripper_state = msg
		self.grippercount+=1

#/callback------


#state retrieval

	def get_rarm_state(self):
		return self.cached_rarm_state

	def get_rgripper_state(self):
		return self.cached_rgripper_state

	def get_rarm_pose(self,cmd_api):
		return cmd_api.get_rarm_fk(self.cached_rarm_state.joint_names,self.cached_rarm_state.actual.positions)

	
#/state-----




#blocking methods for flow control
	def block_rarm_joints(self,positions,timeout):
		exit = rospy.Time.now() + rospy.Duration(timeout)
		while(not(within_tolerance(get_rarm_state().actual.positions,positions) or rospy.time.now>=exit)):
			pass	
		if(rospy.Time.now()>=exit):
			return -1	
		return 1

	"""
	def block_duration(self,duration):
		rospy.sleep(duration)
		
		#exit = rospy.Time.now() + rospy.Duration(duration)
		#while(rospy.Time.now()<=exit):
		#	pass
		#return
		
	"""
#init
	def init_subscribers(self):
		rospy.Subscriber(SUB_RARM_STATE, JointTrajectoryControllerState, self.rarm_callback)
		rospy.Subscriber(SUB_RGRIPPER_STATE, JointControllerState, self.rgripper_callback)
		
		

	def init_all(self):
		self.init_subscribers()
#/init------

"""
if __name__ == '__main__':
                init_all()
                print "State api init success."
"""
