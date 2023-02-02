import rospy

def init(node):
	rospy.init_node(node, anonymous=True)
