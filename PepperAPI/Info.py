from .Publisher import *
from .Subscriber import *

def Request(api, params):

	class Data:
		RLTest = None

	"""
	Subscribe to RLTest
	value: [msg]
	"""
	if api == "RLTest":
		
		def tts_callback(data):
			Data.RLTest = data.data
			rospy.loginfo(Data.RLTest)
		
		_ = TTS_Subscriber("tts_topic", tts_callback, params["listen_once"])

		return Data.RLTest
		
	return


def Send(api, params):


	"""
	Publish test message
	value: [msg]
	"""
	if api == "RLTest":
		msg = params["value"]
		tts_publisher.publish(msg)

	return

