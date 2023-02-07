from .Publisher import *
from naoqi import ALProxy


def Request(api_name, api_params):
	
	# Global parameters
	ROBOT_IP = "192.168.0.102"
	ROBOT_PORT = 9559

	# API callbacks
	"""
	Send message to be said by Pepper
	@param api_params: {
		"value": message to be sent
	}
	"""
	if api_name == "ALTextToSpeech":
		msg = api_params["value"]
		tts = ALProxy("ALTextToSpeech", ROBOT_IP, ROBOT_PORT)
		tts.say(msg)

	return
