# import rospy
# from std_msgs.msg import String

global tts_publisher

"""
Actuator API
"""
class Action:

	@staticmethod
	def Request(api, value):
		
		global tts_publisher
		
		"""
		value: [msg]
		"""
		if api == "ALVisionRecognition":
			msg = value[0]
			tts_publisher.publish(msg)

		return


# ==================================================
"""
Parent publisher class
"""
class Publisher:	
	def __init__(self, topic, node, payload_type):
		self.publisher = rospy.Publisher(topic, payload_type, queue_size=10)
		rospy.init_node(node, anonymous=True)


"""
Child publisher classes
"""
class TTS_Publisher(Publisher, object):
	def __init__(self, topic, node):
		super(TTS_Publisher, self).__init__(topic, node, String)

	def publish(self, message):
		rospy.loginfo("Publish: %s" % message)
		self.publisher.publish(message)

# =======================================================
"""
Instantiate global publishers
"""
def init():
	global tts_publisher

	tts_publisher = TTS_Publisher("tts_topic", "tts_publisher_node")
