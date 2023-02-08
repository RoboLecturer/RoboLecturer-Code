from .Publisher import *
from .Subscriber import *
from api.msg import Coords

def Request(api_name, api_params):

	# Data class defined to store data from ROS Subscribers
	# in return value of Info.Request()
	class Data:
		RLSimpleMsg = None
		RLHandLocation = None
		RLImage = None

	# Callbacks for API
	if api_name == "RLSimpleMsg":
		"""Receive string message
		@param	api_params : dict{
			"listen_once": if True, unregister once msg is received
		}
		@return	msg : String
		"""
		def callback(data):
			Data.RLSimpleMsg = data.data
			rospy.loginfo("RLSimpleMsg: %s" % Data.RLSimpleMsg)
		_ = StringSubscriber("simple_msg_topic", callback,
							 api_params["listen_once"])
		return Data.RLSimpleMsg

	elif api_name == "RLHandLocation":
		"""Receive xy coords as floats
		@return	[x, y] : float[]
		"""
		def callback(data):
			x, y = data.x, data.y
			Data.RLHandLocation = [x, y]
			rospy.loginfo("RLHandLocation: x=%.3f, y=%.3f" % (x, y))
		_ = CoordsSubscriber("hand_location_topic", callback)
		return Data.RLHandLocation

	elif api_name == "RLImage":
		"""Receive image
		@return	image
		"""
		def callback(msg):
			br = CvBridge()
			image = br.imgmsg_to_cv2(msg)
			Data.RLImage = image
			rospy.loginfo("RLImage: image received")
		_ = ImageSubscriber("image_topic", callback)
		return Data.RLImage
	
	return

# =========================================================

def Send(api_name, api_params):

	global simple_msg_publisher, hand_location_publisher, image_publisher

	# Callbacks for API
	if api_name == "RLSimpleMsg":
		"""Send simple string message
		@param	api_params : dict{
			"value": message to be sent
		}
		"""
		msg = api_params["value"]
		simple_msg_publisher.publish(msg)

	elif api_name == "RLHandLocation":
		"""Send list of xy coords
		@param	api_params : dict{
			"value": [x, y]
		}
		"""
		coords = api_params["value"]
		hand_location_publisher.publish(coords)

	elif api_name == "RLImage":
		"""Send image
		@param	api_params : dict{
			"value": image
		}
		"""
		image = api_params["value"]
		image_publisher.publish(image)

	return
