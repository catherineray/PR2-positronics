#! user/bin/python

def move_arm_r(StateAPI,CmdAPI,xyz):
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
		oc.header.frame_id = 'base_link'
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


