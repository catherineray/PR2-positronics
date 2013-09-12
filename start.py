#! /usr/bin/env python
import accept_dowel
import assume_start

def execute(StateAPI,CmdAPI):
	assume_start.execute(StateAPI,CmdAPI)
	accept_dowel.execute(StateAPI,CmdAPI)


