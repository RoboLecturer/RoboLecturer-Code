import rospy, cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from api.msg import CVInfo, State

# Parent subscriber class
class Subscriber:	
	def __init__(self, name, topic, payload_type, callback, listen, log=True):
		self.name = name
		self.topic = topic
		self.subscriber = None
		self.spin = listen

		def finalCallback(data):
			callback(data)
			self.spin -= 1
			if not self.spin:
				self.subscriber.unregister()

		self.subscriber = rospy.Subscriber(topic, payload_type, finalCallback)
		if log:
			self.log()

		# my implementation of rospy.spin()
		while self.spin and not rospy.is_shutdown():
			rospy.rostime.wallsleep(0.5)

	# Log information about subscriber
	def log(self):
		print("[%s] topic=%s, listen=%s" % (self.name, self.topic, self.spin))


# Child subscriber classes
"""receive string"""
class StringSubscriber(Subscriber, object):
	def __init__(self, topic, callback, listen=True):
		super(StringSubscriber, self).__init__(
			"StringSubscriber", 
			topic, 
			String, 
			callback, 
			listen)

"""receive feature info about raised hand or detected face"""
class CVInfoSubscriber(Subscriber, object):
	def __init__(self, topic, callback, listen=True):
		super(CVInfoSubscriber, self).__init__(
			"CVInfoSubscriber", 
			topic, 
			CVInfo,
			callback, 
			listen)

"""receive image"""
class ImageSubscriber(Subscriber, object):
	def __init__(self, topic, callback, listen=True):
		super(ImageSubscriber, self).__init__(
			"ImageSubscriber", 
			topic, 
			Image,
			callback, 
			listen)
		
"""receive state"""
class StateSubscriber(Subscriber, object):
	def __init__(self, topic, callback, listen=True, log=False):
		super(StateSubscriber, self).__init__(
			"StateSubscriber", 
			topic, 
			State,
			callback, 
			listen,
			log)
