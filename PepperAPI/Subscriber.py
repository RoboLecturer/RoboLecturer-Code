import rospy
from std_msgs.msg import String

"""
Parent subscriber class
"""
class Subscriber:	
	def __init__(self, topic, payload_type, callback, listen_once=False):
		self.subscriber = None
		self.spin = True

		def finalCallback(data):
			callback(data)
			if listen_once:
				self.spin = False
				self.subscriber.unregister()

		self.subscriber = rospy.Subscriber(topic, payload_type, finalCallback)
		# implementation of rospy.spin()
		while self.spin:
			rospy.rostime.wallsleep(0.5)

"""
Child subscriber classes
"""
class TTS_Subscriber(Subscriber, object):
	def __init__(self, topic, callback, listen_once=False):
		super(TTS_Subscriber, self).__init__(topic, String, callback, listen_once)

"""
Initiate subscribers
"""
# def init(node):
# 	rospy.init_node(node, anonymous=True)
