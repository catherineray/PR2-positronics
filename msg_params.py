def set_msg_params(str, user):
	output = check_output("rosmsg show" + str)
	#space words with cap letters, word after in diff array 
	#r.append([str + "." + out[i] + " = " + user[i] for i in range(len(output))])
	return ' '.join(output)
	

def check_output(command):
	process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
	output = process.communicate()
	retcode = process.poll()
	if retcode:
		raise subprocess.CalledProcessError(retcode, command, output=output[0])
	return output[0].split('\n') 




