import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Received Data")
    pub = rospy.Publisher('take_control', String, queue_size=10)
    pub.publish("Forwarder published a message")

def callback_num_slides(data):
    print(data)
    rospy.loginfo("Received Data")
    pub = rospy.Publisher('num_slides', String, queue_size=10)
    pub.publish(data)

def callback_slides(data):
    print(data)
    rospy.loginfo("Received Data")
    pub = rospy.Publisher('slides', String, queue_size=10)
    pub.publish(data)

def callback2(data):
    print("callback2")

def listener():
    rospy.init_node("web_forwarder_node", xmlrpc_port=45100, tcpros_port=45101)
    rospy.Subscriber('take_control_forwarder', String, callback)
    rospy.Subscriber('num_slides_forwarder', String, callback_num_slides)
    rospy.Subscriber('slides_forwarder', String, callback_slides)   
    rospy.spin()

# def forwarder():
#      pub = rospy.Publisher('take_control', String, queue_size=10)
#      rospy.init_node('web_forwarder_node', xmlrpc_port=45100, tcpros_port=45101) #port important! Must be same as in Docker run
#      rate = rospy.Rate(1)
#      rospy.loginfo("Publisher Node Started, now publishing messages")
#      while not rospy.is_shutdown():
#          msg = "Hello I'm Pepper! - %s" % rospy.get_time()
#          pub.publish(msg)
#          rate.sleep()


if __name__ == '__main__':
    listener()