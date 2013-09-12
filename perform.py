#! /usr/bin/env python
import playback_fixed as playback
import seek_hole
from staterecording import StateRecording
import assume_start

def execute(StateAPI,CmdAPI, recording):
	playback.execute_forward(StateAPI,CmdAPI,recording)
	seek_hole.execute(StateAPI,CmdAPI)
