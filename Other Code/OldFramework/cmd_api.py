#! /usr/bin/env python
#The python wrapper/API for PR2 running ROS


#publishing topics
PUB_RARM_COMMAND = "r_arm_controller/command"
PUB_RGRIPPER_COMMAND = "r_gripper_controller/command"
PUB_RGRIPPER_STATUS = "r_gripper_controller/gripper_action/status"
PUB_RGRIPPER_RESULT = "r_gripper_controller/gripper_action/result"


#-----------------
def init_publishers:
 	global publisher_rgripper    
	global publisher_rarm 
	global publisher_rgripper_result
	global publisher_rgripper_status 

    	publisher_rgripper = rospy.Publisher(PUB_RGRIPPER_COMMAND, Pr2GripperCommand)   
   	publisher_rarm  = rospy.Publisher(PUB_RARM_COMMAND, JointTrajectory)

	#Describes why the gripper stopped (stalled or reached the final position). 
	publisher_rgripper_result = rospy.Publisher(PUB_RGRIPPER_RESULT, Pr2GripperActionResult)
 
	#Provides status information on the goals that are sent to the action
	publisher_rgripper_status (actionlib_msgs/GoalStatusArray) 


def init_all():
	init_publishers()

#-----------------Arm Wrapper
def goToPoint(self, point, orientation=[0,0,0,1], frame_id = None, link = None):
       """ Go to a cartisian point [x, y, z] in frame frame_id
           Link specifies the URDF link that must move to that coordinate
           With no link given, it defaults to the l or r _wrist_roll_link
       """
       if (frame_id == None):
           frame_id  = "base_link"
       if (link == None):
           link = self.controller[1] + '_wrist_roll_link'          
       pose = self.makePose(point, orientation, frame_id)
       try:
           ik  = self.getIK(pose, link, self.getJointAngles())
           self.gotoAngle(ik.solution.joint_state.position)
       except:
           print "Could not move to that positon"

def genArmPose(xyz, orientation, frameID):
       """Pose stamped messages"""
       pose = geometry_msgs.msg.PoseStamped()
       pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz[0], xyz[1], xyz[2]
       pose.pose.orientation.x = orientation[0]
       pose.pose.orientation.y = orientation[1]
       pose.pose.orientation.z = orientation[2]
       pose.pose.orientation.w = orientation[3]
       pose.header.frame_id = frameID
       pose.header.stamp = rospy.Time.now()
       return pose


"""
Currently in the process of implementing pre-existant gripper sensor stack code
"""
#Gripper Wrapper for pr2_object_manipulation



""""

#-----------------Gripper Wrapper
def genGripperPose(target,effort):
"""Gripper pose wrapper"""
	gpose = pr2_controllers.msg.PR2GripperCommandGoal()
	gpose.command.position = position
	gpose.command.max_effort =  max_effort
	return gpose

def genGripperHoldForce(holdForce)
"""Hold something with constant force in the gripper"""
	squeeze = pr2_controllers.msg.PR2GripperForceServoGoal()
	squeeze.command.fingertip_force = holdForce
	return squeeze

"""
