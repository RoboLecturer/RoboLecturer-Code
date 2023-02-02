from .Publisher import *

def Request(api, value):
	
	"""
	Send message to be said by Pepper
	value: [msg]
	"""
	if api == "ALTextToSpeech":
		msg = value[0]
		tts_publisher.publish(msg)

	return
