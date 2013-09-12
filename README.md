peg-in-hole
===========
_Catherine Ray and Gregory Colella (mentor: Roxana Leontie)_<br>
George Washington University, Robotics Lab: Positronics Division<br>
Human-Computer Interaction Intern Project, Summer 2012<br><br>

__Program for PR2 using ROS stacks and Python. PR2 completes a task 
moving a dowel into a hole (using only force proprioception)
as a response to dynamic stimuli.__<br><br>


After downloading the ROS stacks and this package, the following commands will execute the autonomous task completion in gazebo:
```
  roslaunch pr2_gazebo pr2_empty_world.launch 
  roslaunch pr2_3dnav both_arms_navigation.launch
  rosservice call /environment_server/set_planning_scene_diff '{}'
  rosrun peg_in_hole DiagExecute.py
```
