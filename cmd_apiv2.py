#! /usr/bin/env python
import roslib; roslib.load_manifest("peg_in_hole")
import actionlib
import rospy
from actionlib_msgs.msg import *
import geometry_msgs.msg
import kinematics_msgs.msg
import kinematics_msgs.srv
import arm_navigation_msgs.msg
from arm_navigation_msgs.msg import *
import arm_navigation_msgs.srv
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import std_msgs.msg
from sensor_msgs.msg import JointState
from pr2_controllers_msgs.msg import *
from pr2_gripper_sensor_msgs.msg import *


#----publishing topics
rarm_command = "r_arm_controller/command"
fk = '/pr2_right_arm_kinematics/get_fk'
grab_pub ='r_gripper_sensor_controller/grab'
action_pub = 'r_gripper_controller/gripper_action'
#----/publishing topics

class Cmd_API:
	def __init__(self):
		self.publisher_rarm = rospy.Publisher(rarm_command, JointTrajectory)
		#-----
		self.fk_service = rospy.ServiceProxy(fk, kinematics_msgs.srv.GetPositionFK)
		#------		
		self.grab_ac = actionlib.SimpleActionClient(grab_pub, PR2GripperGrabAction)
		self.gripper_client = actionlib.SimpleActionClient(action_pub, Pr2GripperCommandAction)
	
	def rarm_assumeJointPositions(self, joint_names,joint_positions):
        	cmd = trajectory_msgs.msg.JointTrajectory()
       	 	cmd.header.stamp = rospy.Time.now()
        	cmd.joint_names = joint_names
        	cmd.points.append(JointTrajectoryPoint())
        	cmd.points[0].time_from_start = rospy.Duration(.125)
        	cmd.points[0].positions = joint_positions
        	self.publisher_rarm.publish(cmd)

	# Moves right arm to given coordinates. (WRT torso_lift_link) 
	def move_arm_r(self, xyz):
		move_arm = actionlib.SimpleActionClient('move_right_arm', MoveArmAction)
		move_arm.wait_for_server()
		rospy.logout('Connected to server')
		goalA = arm_navigation_msgs.msg.MoveArmGoal()
		goalA.motion_plan_request.group_name = 'right_arm'
		goalA.motion_plan_request.num_planning_attempts = 1
		goalA.motion_plan_request.planner_id = ''
		goalA.planner_service_name = 'ompl_planning/plan_kinematic_path'
		goalA.motion_plan_request.allowed_planning_time = rospy.Duration(5.0)
		## set and request position constraint
		pc = PositionConstraint()
		pc.header.stamp = rospy.Time.now()
		pc.header.frame_id = 'torso_lift_link'
		pc.link_name = 'r_wrist_roll_link'
		pc.position.x, pc.position.y, pc.position.z  = xyz[0], xyz[1], xyz[2]
		pc.constraint_region_shape.type = Shape.BOX
		pc.constraint_region_shape.dimensions = [0.02, 0.02, 0.02]
		pc.constraint_region_orientation.x = 1.0
		goalA.motion_plan_request.goal_constraints.position_constraints.append(pc)
		## set and request orientation constraint
		oc = OrientationConstraint()
		oc.header.stamp = rospy.Time.now()
		oc.header.frame_id = 'base_link' #gripper oriented parallel to floor
		oc.link_name = 'r_wrist_roll_link' 
		oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w = 1.0, 0.0, 0.0, 0.0
		oc.absolute_roll_tolerance, oc.absolute_pitch_tolerance, oc.absolute_yaw_tolerance, oc.weight = 0.04, 0.04, 0.04, 1.0
    		goalA.motion_plan_request.goal_constraints.orientation_constraints.append(oc)
		move_arm.send_goal(goalA)
		finished_within_time = move_arm.wait_for_result()
		if not finished_within_time:
			move_arm.cancel_goal()
			rospy.logout('Timed out achieving goal A')
		else:
			state = move_arm.get_state()
			if state == GoalStatus.SUCCEEDED:
				rospy.logout('Action finished with SUCCESS')
			else:
				rospy.logout('Action failed')

#----gripper
	def grab(self, gain=0.03):
		print "Performing Gripper Grab"
		self.grab_ac.wait_for_server()
		self.grab_ac.send_goal(PR2GripperGrabGoal(PR2GripperGrabCommand(gain)))
            	self.grab_ac.wait_for_result(rospy.Duration(20))

	def gripper_action(self, position, max_effort):
		self.gripper_client.wait_for_server()
		cmd = pr2_controllers_msgs.msg.Pr2GripperCommand()
		cmd.position = position 
		cmd.max_effort = max_effort
		self.gripper_client.send_goal(Pr2GripperCommandGoal(cmd))
		self.gripper_client.wait_for_result()
#----/gripper


#---- FK solver
# Documentation on MultiDOFJointState (written in C++)
# http://mirror.umd.edu/roswiki/doc/electric/api/arm_navigation_msgs/html/MultiDOFJointState_8h_source.html

	def get_rarm_fk(self, joint_names, joint_positions, frame_id = 'torso_lift_link', link_name = None):
#runs forward kinematics on (a set of 7) joint angles, returns PoseStamped msg
		if (link_name == None):
			link_name = 'r_wrist_roll_link'
		header = rospy.Header()
		header.stamp = rospy.get_rostime()
		header.frame_id = frame_id
		joint_state = JointState(header, joint_names, joint_positions, [], [])
		response = self.fk_service(header, [link_name], RobotState(joint_state, MultiDOFJointState())) 
		return response.pose_stamped[0]
#----/FK solver
