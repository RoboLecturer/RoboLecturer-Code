import rospy
from std_msgs.msg import String

# Parent subscriber class
class Subscriber:	
	def __init__(self, topic, payload_type, callback, listen_once=False):
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
		print("[Subscriber] topic=%s, spin=%s" % (self.topic, self.spin))

# Child subscriber classes
class SimpleMsgSubscriber(Subscriber, object):
	def __init__(self, topic, callback, listen_once=False):
		super(SimpleMsgSubscriber, self).__init__(topic, String, callback, listen_once)

	def log(self):
		print("[SimpleMsgSubscriber] topic=%s, spin=%s" % (self.topic, self.spin))

