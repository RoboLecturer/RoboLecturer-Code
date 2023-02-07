from .Publisher import *
from .Subscriber import *

def Request(api_name, api_params):

	# Data class defined to store data from ROS Subscribers 
	# in return value of Info.Request()
	class Data:
		RLSimpleMsg = None

	# Callbacks for API

	"""
	Receive string message
	@param api_params: {
		"listen_once": if True, unregister once msg is received
	}
	"""
	if api_name == "RLSimpleMsg":
		
		def callback(data):
			Data.RLSimpleMsg = data.data
			rospy.loginfo(Data.RLSimpleMsg)
		
		_ = SimpleMsgSubscriber("simple_msg_topic", callback, api_params["listen_once"])

		return Data.RLSimpleMsg
		
	return


def Send(api_name, api_params):

	# Callbacks for API

	"""
	Publish simple string message
	@param api_params: {
		"value": message to be sent
	}
	"""
	if api_name == "RLSimpleMsg":
		msg = api_params["value"]
		simple_msg_publisher.publish(msg)

	return

