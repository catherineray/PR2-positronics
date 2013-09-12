class StateRecording:
	def __init__(self,jointnames):
		self.joint_names = jointnames
		self.recorded_states = []
	
	def add_states(self,newrecording):
		self.recorded_states.append(newrecording)

	def get_states(self):
		return self.recorded_states

	def copy(self):
		out = StateRecording(self.joint_names)
		for state in self.recorded_states:
			out.add_states(state)
		return out
	
	def reverse(self):
		self.recorded_states.reverse()
	
	def get_names(self):
		return self.joint_names

	def get_dicts(self):
		"""Get an array of dictionaries, each representing each joint's position throughout the recording."""
		names = self.joint_names
		outdict = []
		for state in self.recorded_states:
			outdict.append(zip(names,state))
		return outdict
