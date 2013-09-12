roslaunch pr2_gazebo pr2_empty_world.launch 

roslaunch pr2_3dnav both_arms_navigation.launch

rosservice call /environment_server/set_planning_scene_diff '{}'

rosrun peg_in_hole DiagExecute.py
