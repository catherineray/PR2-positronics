#! /usr/bin/env python
import move_arm_.py
import gripper_.py

rospy._init_node("general_callback")

query statelistener
if pressure_felt msg recieved 
	translator.translateGoal('goalA')
else
	update listener
	loop until pressure_felt

if grasp returns success
	translator.translateGoal('goalA')

if armmoved = true 
 	lock_arm_.lock_arm('cmd')

else update error positions
	query error positions
while arm_moved = true 
record trajectory in array

arm _moved = false time stamp 30 sec since arm moved = true
send command to translator to record current pose in move_arm format

query
if translator = sucess
	translator.translateGoal('goalA')

if goalA = success
	translator.translateGoal('goalB') #recorded trajectory

if goal B = sucesss
	listen to gripper sensors 
	query table_contact from listener

while table_contact = false 
	cmd move gripper to table (keep orientation parallel)
	send message to translator
	update sensor msgs, wait until table_contact = true

while table_contact = true
	translator.translateGoal('goalC') #do sinusoidal arm motion with locked coordinate gripper 	
	{if sudden table contact = false check pr2_sensor_msgs header timestamp) send request to translator to lock arm 

send listener command to wait for tapin
	if tap
	gripper_.gripperAction('release')

else 
	loop counter until counter = 100
	send cmd release to gripper grasp

if grasp(release) = success
	translator.translateGoal('goalA')

spin()

return 0







