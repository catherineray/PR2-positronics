import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('pr2_controllers_msgs')
roslib.load_manifest('geometry_msgs')

import rospy
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import *
http://brown-ros-pkg.googlecode.com/svn/trunk/experimental/pr2_remotelab/ik_control/bin/arm.py

rospy.init_node('move_arm', anonymous=True)
"""
client = actionlib.SimpleActionClient(
    '/r_arm_controller/point_head_action', MoveArmAction)
client.wait_for_server()

g = MoveArmGoal()
g.target.header.frame_id = 'torso_lift_link'
g.target.point.x = 
g.target.point.y = 
g.target.point.z = 
g.min_duration = rospy.Duration(1.0)

client.send_goal(g)
client.wait_for_result()

if client.get_state() == GoalStatus.SUCCEEDED:
    print "Succeeded"
else:
    print "Failed"

"""

def move_arm(ac, grasp_pose, trajectory):

    if trajectory == None:
        trajectory = JointTrajectory()

    goal = ReactiveGraspGoal()
    goal.final_grasp_pose = grasp_pose
    goal.trajectory = trajectory
    goal.collision_support_surface_name = "table"

    ac.send_goal(goal)    
    ac.wait_for_result()
    result = ac.get_result()
    print "reactive approach result:", result

    return result













def poseConstraintToPositionOrientationConstraints(pose_constraint):
    position_constraint = PositionConstraint()
    orientation_constraint = OrientationConstraint()
    position_constraint.header = pose_constraint.header
    position_constraint.link_name = pose_constraint.link_name
    position_constraint.position = pose_constraint.pose.position
    position_constraint.constraint_region_shape.type = 0
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.x)
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.y)
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.z)

    position_constraint.constraint_region_orientation.x = 0.0
    position_constraint.constraint_region_orientation.y = 0.0
    position_constraint.constraint_region_orientation.z = 0.0
    position_constraint.constraint_region_orientation.w = 1.0

    position_constraint.weight = 1.0

    orientation_constraint.header = pose_constraint.header
    orientation_constraint.link_name = pose_constraint.link_name
    orientation_constraint.orientation = pose_constraint.pose.orientation
    orientation_constraint.type = pose_constraint.orientation_constraint_type

    orientation_constraint.absolute_roll_tolerance = pose_constraint.absolute_roll_tolerance
    orientation_constraint.absolute_pitch_tolerance = pose_constraint.absolute_pitch_tolerance
    orientation_constraint.absolute_yaw_tolerance = pose_constraint.absolute_yaw_tolerance
    orientation_constraint.weight = 1.0

    return (position_constraint, orientation_constraint)


def addGoalConstraintToMoveArmGoal(pose_constraint, move_arm_goal):
    position_constraint, orientation_constraint = poseConstraintToPositionOrientationConstraints(pose_constraint);
    move_arm_goal.motion_plan_request.goal_constraints.position_constraints.append(position_constraint)
    move_arm_goal.motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint)


rospy.init_node('place_hand', anonymous=True)
side = rospy.get_param("~side", "right")

move_arm = actionlib.SimpleActionClient("move_" + side + "_arm", MoveArmAction)

dynamixel_namespace = '/dynamixel_controller'
dynamixels = rospy.get_param(dynamixel_namespace + '/dynamixels', dict())

servo_torque_enable = list()

for name in sorted(dynamixels):
    torque_enable_service = dynamixel_namespace + '/' + name + '_controller/torque_enable'
    rospy.wait_for_service(torque_enable_service)  
    servo_torque_enable.append(rospy.ServiceProxy(torque_enable_service, TorqueEnable))

move_arm.wait_for_server()
rospy.loginfo("Connected to move_"+side+"_arm server")

goal = MoveArmGoal()

goal.motion_plan_request.group_name = side + "_arm"
goal.motion_plan_request.num_planning_attempts = 1
goal.motion_plan_request.allowed_planning_time = rospy.Duration(10.0)

goal.motion_plan_request.planner_id = ""
goal.planner_service_name = "ompl_planning/plan_kinematic_path"

desired_pose = SimplePoseConstraint()
desired_pose.header.frame_id = "lower_torso_link"
desired_pose.link_name = side + "_hand_link";
desired_pose.pose.position.x = 0.248939291788;
desired_pose.pose.position.y =  0.00880423979233;
desired_pose.pose.position.z = -0.0346550990121;

desired_pose.pose.orientation.x = 0.317800304267;
desired_pose.pose.orientation.y = -0.541804275305;
desired_pose.pose.orientation.z = 0.299045455395;
desired_pose.pose.orientation.w = 0.71834734598;

desired_pose.absolute_position_tolerance.x = 0.05;
desired_pose.absolute_position_tolerance.y = 0.05;
desired_pose.absolute_position_tolerance.z = 0.05;

desired_pose.absolute_roll_tolerance = 0.2;
desired_pose.absolute_pitch_tolerance = 0.2;
desired_pose.absolute_yaw_tolerance = 0.2;

addGoalConstraintToMoveArmGoal(desired_pose, goal)

finished_within_time = False
move_arm.send_goal(goal) 
finished_within_time = move_arm.wait_for_result(rospy.Duration(200));
if not finished_within_time:
    move_arm.cancel_goal()
    rospy.loginfo("Timed out achieving goal")
else:
    state = move_arm.get_state()
    if state == GoalStatus.SUCCEEDED:
      rospy.loginfo("Action succeeded!")
    else:
      rospy.loginfo("Action failed with error code: " + str(state))

# Relax all servos to give them a rest.
for torque_enable in servo_torque_enable:
    torque_enable(False)

print "Finished"
