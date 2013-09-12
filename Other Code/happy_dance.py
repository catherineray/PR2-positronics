#! /usr/bin/env python
from arm import Arm
#from seek_hole import search_table
import play_music as play
from assume_start import *

"""
def execute():
	dance()

def dance():
	if search_table() == "stop":
		hammertime(win)
	else:
		hammertime(lose)		
"""

def hammertime(mission_status):
	if mission_status == "win":
		play.playWAV("super.wav")
	elif mission_status == "lose":
		play.playWAV("worry.wav")
		assume_start.assumeStartGoal()
	else: 
		print "Error, unknown mission status. Would you like me to go and stick my head in a bucket of water?"
		
		
hammertime("win")

	
