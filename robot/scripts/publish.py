import rospy
from std_msgs.msg import String

def talk_to_me():
    pub = rospy.Publisher('talking', String, queue_size=10)
    rospy.init_node('publisher_node', xmlrpc_port=45100, tcpros_port=45101) #port important! Must be same as in Docker run
    rate = rospy.Rate(1)
    rospy.loginfo("Publisher Node Started, now publishing messages")
    while not rospy.is_shutdown():
        msg = "Hello I'm Pepper! - %s" % rospy.get_time()
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    talk_to_me()
    