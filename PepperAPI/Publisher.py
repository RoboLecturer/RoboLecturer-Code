import rospy
from std_msgs.msg import String

# Parent publisher class
class Publisher:
	def __init__(self, topic, payload_type):
		self.topic = topic
		self.publisher = rospy.Publisher(self.topic, payload_type, queue_size=10)
		self.log()

	# Log info about publisher
	def log(self):
		print("[Publisher] topic=%s" % self.topic)

# Child publisher classes
class SimpleMsgPublisher(Publisher, object):
	def __init__(self, topic):
		super(SimpleMsgPublisher, self).__init__(topic, String)

	def publish(self, message):
		self.publisher.publish(message)
		rospy.loginfo("Publish: %s" % message)

	def log(self):
		print("[SimpleMsgPublisher] topic=%s" % self.topic)


# Initiate publishers
simple_msg_publisher = SimpleMsgPublisher("simple_msg_topic")
