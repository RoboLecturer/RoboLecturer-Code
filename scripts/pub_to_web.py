import rospy
from std_msgs.msg import String
import sys

if __name__ == "__main__":
	rospy.init_node("me", xmlrpc_port=45100, tcpros_port=45101, disable_signals=True)
	quiz_pub = rospy.Publisher("start_quiz", String, queue_size=10)
	slide_pub = rospy.Publisher("change_slide", String, queue_size=10)
	try:
		# slide_pub.publish("increment|0")
		while True:
			quiz_pub.publish("1")
			# slide_pub.publish("increment|0")
			rospy.Rate(1).sleep()
	except:
		sys.exit()
