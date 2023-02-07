import rospy
from std_msgs.msg import String
from api.msg import Coords

# Parent subscriber class
class Subscriber:	
	def __init__(self, name, topic, payload_type, callback, listen_once=False):
		self.name = name
		self.topic = topic
		self.subscriber = None
		self.spin = True

		def finalCallback(data):
			callback(data)
			if listen_once:
				self.spin = False
				self.subscriber.unregister()

		self.subscriber = rospy.Subscriber(topic, payload_type, finalCallback)
		self.log()
		# my implementation of rospy.spin()
		while self.spin and not rospy.is_shutdown():
			rospy.rostime.wallsleep(0.5)

	# Log information about subscriber
	def log(self):
		print("[%s] topic=%s, spin=%s" % (self.name, self.topic, self.spin))


# Child subscriber classes
"""receive simple string msg"""
class StringSubscriber(Subscriber, object):
	def __init__(self, topic, callback, listen_once=False):
		super(StringSubscriber, self).__init__(
			"StringSubscriber", 
			topic, 
			String, 
			callback, 
			listen_once)

"""receive array of xy float coordinates"""
class CoordsSubscriber(Subscriber, object):
	def __init__(self, topic, callback):
		super(CoordsSubscriber, self).__init__(
			"CoordsSubscriber", 
			topic, 
			Coords,
			callback, 
			listen_once=True)
