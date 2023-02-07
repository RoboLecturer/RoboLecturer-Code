from .Publisher import *
from .Subscriber import *
from api.msg import Coords

def Request(api_name, api_params):

	# Data class defined to store data from ROS Subscribers 
	# in return value of Info.Request()
	class Data:
		RLSimpleMsg = None
		RLHandLocation = None

	# Callbacks for API

	"""
	Receive string message
	@param	api_params : dict{
		"listen_once": if True, unregister once msg is received
	}
	@return	msg : String
	"""
	if api_name == "RLSimpleMsg":
		def callback(data):
			Data.RLSimpleMsg = data.data
			rospy.loginfo("RLSimpleMsg:", Data.RLSimpleMsg)
		_ = StringSubscriber("simple_msg_topic", callback, api_params["listen_once"])
		return Data.RLSimpleMsg

	# """
	# Receive xy coords as floats
	# @return	[x, y] : float[]
	# """
	elif api_name == "RLHandLocation":
		def callback(data):
			x, y = data.x, data.y
			Data.RLHandLocation = [x,y]
			rospy.loginfo("RLHandLocation: x=%.3f, y=%.3f" % (x,y))
		_ = CoordsSubscriber("hand_location_topic", callback)
		return Data.RLHandLocation
		
	return


def Send(api_name, api_params):

	# Callbacks for API

	"""
	Send simple string message
	@param	api_params : dict{
		"value": message to be sent
	}
	"""
	if api_name == "RLSimpleMsg":
		msg = api_params["value"]
		simple_msg_publisher.publish(msg)

	# """
	# Send list of xy coords
	# @param	api_params : dict{
	# 	"value": [x, y]
	# }
	# """
	elif api_name == "RLHandLocation":
		coords = api_params["value"]
		hand_location_publisher.publish(coords)

	return

