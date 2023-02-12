import rospy, cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from api.msg import CVInfo

# Parent subscriber class
class Subscriber:	
	def __init__(self, name, topic, payload_type, callback, listen_once=False):
		self.name = name
		self.topic = topic
		self.subscriber = None
		self.listen_once = listen_once
		self.spin = True

		def finalCallback(data):
			callback(data)
			if listen_once:
				self.spin = False
				self.subscriber.unregister()

		self.subscriber = rospy.Subscriber(topic, payload_type, finalCallback)
		# self.log()
		# my implementation of rospy.spin()
		while self.spin and not rospy.is_shutdown():
			rospy.rostime.wallsleep(0.5)

	# Log information about subscriber
	def log(self):
		print("[%s] topic=%s, listen_once=%s" % (self.name, self.topic, self.listen_once))


# Child subscriber classes
"""receive simple string msg"""
class StringSubscriber(Subscriber, object):
	def __init__(self, topic, callback, listen_once=True):
		super(StringSubscriber, self).__init__(
			"StringSubscriber", 
			topic, 
			String, 
			callback, 
			listen_once)

"""receive feature info about raised hand or detected face"""
class CVInfoSubscriber(Subscriber, object):
	def __init__(self, topic, callback):
		super(CVInfoSubscriber, self).__init__(
			"CVInfoSubscriber", 
			topic, 
			CVInfo,
			callback, 
			listen_once=True)

"""receive image"""
class ImageSubscriber(Subscriber, object):
	def __init__(self, topic, callback):
		super(ImageSubscriber, self).__init__(
			"ImageSubscriber", 
			topic, 
			Image,
			callback, 
			listen_once=True)
