import rospy
from std_msgs.msg import String

"""
Parent publisher class
"""
class Publisher:	
	def __init__(self, topic, payload_type):
		self.publisher = rospy.Publisher(topic, payload_type, queue_size=10)

"""
Child publisher classes
"""
class TTS_Publisher(Publisher, object):
	def __init__(self, topic):
		super(TTS_Publisher, self).__init__(topic, String)

	def publish(self, message):
		self.publisher.publish(message)
		rospy.loginfo("Publish: %s" % message)


"""
Initiate publishers
"""
tts_publisher = TTS_Publisher("tts_topic")

# def init(node):
# 	rospy.init_node(node, anonymous=True)
