def lock_arm(lock):
"""This this locks the arm if it receives a lock request"""
	global pub
	
	if lock in ['cmd']:
	"""Reuse of trajectory_lock.py code by Vijay Pradeep"""	
	# Copy our current state into the commanded state
        	cmd = trajectory_msgs.msg.JointTrajectory()
        	cmd.header.stamp = msg.header.stamp
        	cmd.joint_names = msg.joint_names
		cmd.points.append( trajectory_msgs.msg.JointTrajectoryPoint())
        	cmd.points[0].time_from_start = rospy.Duration(.125)
        	cmd.points[0].positions = msg.actual.positions
        	pub.publish(cmd)
    	else:
        	print "Lock command not recognized"


#alter publisher/subscriber?
global pub
pub = rospy.Publisher("command", trajectory_msgs.msg.JointTrajectory)

rospy.Subscriber("state", pr2_controllers_msgs.msg.JointTrajectoryControllerState, callback)

rospy.spin()
