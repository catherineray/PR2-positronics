#! /usr/bin/env python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('pr2_controllers_msgs')

import rospy
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *

rospy.init_node('gripper', anonymous=True)


"""
Publishes if successful to listener
Subscribes to /state and translator messages
"""
"""This opens the right gripper if it recieves a release command from the translator"""

def gripperAction(grip_command):

	if grip_command in ['release']:
		client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
		client.wait_for_server()

		##open gripper
		client.send_goal(Pr2GripperCommandGoal(
        	Pr2GripperCommand(position = 0.4, max_effort = -1)))
		client.wait_for_result()

		result = client.get_result()
		did = []
		if client.get_state() != GoalStatus.SUCCEEDED:
    			did.append("failed")
		else:
    			if result.stalled: did.append("stalled")
    			if result.reached_goal: did.append("reached goal"); return release_success
		print ' and '.join(did)

	elif grip_command in ['grasp']:
		client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
		client.wait_for_server()

		##close gripper (position = ?)
		client.send_goal(Pr2GripperCommandGoal(
        	Pr2GripperCommand(position = 0.0, max_effort = -1)))
		client.wait_for_result()

		result = client.get_result()
		did = []
		if client.get_state() != GoalStatus.SUCCEEDED:
    			did.append("failed")
		else:
    			if result.stalled: did.append("stalled")
    			if result.reached_goal: did.append("reached goal"); return release_success
		print ' and '.join(did)

	else:
		print "Error, unknown grip_command. Known commands are [release, grasp]."
		
	#send success message to listener/callback?
"""

pub.publish(sucess)
    else:
        print "Small: %.4f" % max_error

global pub
pub = rospy.Publisher("success", gripperstate.msg.StateMessage)

rospy.Subscriber("command", pr2_controllers_msgs.msg.Pr2GripperState, release)
"""

#keep orientation of gripper constantly parallel to floor

"""
client = actionlib.SimpleActionClient('torso_controller/position_joint_action',   SingleJointPositionAction)
client.wait_for_server()

client.send_goal(SingleJointPositionGoal(position = 0.2))
client.wait_for_result()
if client.get_state() == GoalStatus.SUCCEEDED:
    print "Success"
"""




