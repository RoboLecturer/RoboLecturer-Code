import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Received Data", data.data)
    pub = rospy.Publisher('take_control', String, queue_size=10)
    pub.publish("Forwarder published a message")

def listener():
    rospy.init_node("web_forwarder_node", xmlrpc_port=45100, tcpros_port=45101)
    rospy.Subscriber('take_control_forwarder', String, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptExecption:
        pass