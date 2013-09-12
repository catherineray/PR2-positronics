#! /usr/bin/env python
"""https://kforge.ros.org/Sushi/www/pr2_python/pr2_python.gripper-pysrc.html"""
import roslib 
roslib.load_manifest('pr2_python') 
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('pr2_controllers_msgs')
import rospy 
import actionlib 
#import pr2_python.exceptions as ex 
import pr2_controllers_msgs.msg as pr2c 
import actionlib_msgs.msg as am
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *

"""
Defines the Gripper class for controlling the PR2 gripper. 
"""

#TODO: Implement pr2_object_manipulation stack for more accurate grasping

class Gripper(object):

	def __init__(self, 'right_arm'):
 	#assert side in ['left_arm', 'right_arm'] 
 	#self._side = side 
 	#action_name = '{0}_gripper_controller/gripper_action'.\ 
 	#format('l' if side=='left_arm' else 'r')
		action_name = '{0}_gripper_conroller/gripper_action'.\
		self.acmd = actionlib.SimpleActionClient(action_name, 	pr2c.Pr2GripperCommandAction) 
		rospy.loginfo("Waiting for action server {0}...".format(action_name)) 
 		self.acmd.wait_for_server() 
 		rospy.loginfo("Connected to action server {0}".format(action_name))

	def openGripper(self, max_effort=-1):
		self.move(0.085, max_effort)

	def closeGripper(self, max_effort=-1): 
 		self.move(0.0, max_effort)

	def moveGripper(self, position, max_effort=-1):
		goal = pr2c.Pr2GripperCommandGoal(pr2c.Pr2GripperCommand(position=position, max_effort=max_effort)) 
 		self.acmd.send_goal(goal) 
 		rospy.loginfo("Sending goal to gripper and waiting for result...") 
 		self.acmd.wait_for_result() 
 		rospy.loginfo("Gripper action returned") 
 		if self.acmd.get_state() != am.GoalStatus.SUCCEEDED: 
 			#raise ex.ActionFailedError()
			ROS_ERROR "ActionFailed"




""" Old close method
def gripperClose():
	gripperClose = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
	gripperClose.wait_for_server()

	##open gripper
	gripperClose.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position = 0.4, max_effort = -1)))
	gripperClose.wait_for_result()

	result = gripperClose.get_result()
	did = []
	if gripperClose.get_state() != GoalStatus.SUCCEEDED:
    		did.append("failed")
	else:
    		if result.stalled: did.append("stalled")
    		if result.reached_goal: did.append("reached goal"); return release_success
		print ' and '.join(did)
"""

""" Old grasp method

def gripperGrasp():
	gripperGrasp = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
	gripperGrasp.wait_for_server()
	
	##close gripper
	gripperGrasp.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position = 0.4, max_effort = -1)))
	gripperGrasp.wait_for_result()
	result = gripperGrasp.get_result()
	did = []
	if gripperGrasp.get_state() != GoalStatus.SUCCEEDED:
    		did.append("failed")
	else:
    		if result.stalled: did.append("stalled")
    		if result.reached_goal: did.append("reached goal"); return release_success
		print ' and '.join(did)


"""




