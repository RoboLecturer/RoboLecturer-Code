import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from api.msg import Coords
from cv_bridge import CvBridge

# Parent publisher class
class Publisher:
	def __init__(self, name, topic, payload_type):
		self.name = name
		self.topic = topic
		self.publisher = rospy.Publisher(self.topic, payload_type, queue_size=10)
		self.log()

	# Log info about publisher
	def log(self):
		print("[%s] topic=%s" % (self.name, self.topic))

# Child publisher classes
"""publish string msg"""
class StringPublisher(Publisher, object):
	def __init__(self, topic):
		super(StringPublisher, self).__init__(
			"StringPublisher",
			topic, 
			String)

	def publish(self, message):
		self.publisher.publish(message)
		rospy.loginfo("Publish: %s" % message)


"""publish array of xy float coordinates"""
class CoordsPublisher(Publisher, object):
	def __init__(self, topic):
		super(CoordsPublisher, self).__init__(
			"CoordsPublisher",
			topic,
			Coords)

	def publish(self, coords):
		msg = Coords()
		msg.x, msg.y = coords[0], coords[1]
		self.publisher.publish(msg)


"""publish image"""
class ImagePublisher(Publisher, object):
	def __init__(self, topic):
		super(ImagePublisher, self).__init__(
			"ImagePublisher",
			topic,
			Image)

	def publish(self, image):
		br = CvBridge()
		self.publisher.publish(br.cv2_to_imgmsg(image))
		rospy.loginfo("Published image")

# ==============================================================
# Initiate publishers
simple_msg_publisher = StringPublisher("simple_msg_topic")
hand_location_publisher = CoordsPublisher("hand_location_topic")
image_publisher = ImagePublisher("image_topic")

# ActionRequest publishers
tts_publisher = StringPublisher("tts_topic")
audio_player_publisher = StringPublisher("audio_player_topic")
