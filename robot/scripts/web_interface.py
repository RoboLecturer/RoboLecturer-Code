import rospy
from std_msgs.msg import String

def change_slide_callback(data):
    rospy.loginfo("Received slide change request.", data.data)

def initiate_quiz_callback(data):
    rospy.loginfo("Received trigger to start a quiz.", data.data)

def toggle_control_callback(data):
    rospy.loginfo("Received trigger to give robot control", data.data)


def start_global_listener():
    '''
    Creates subscribers which listen to commands from other modules.
    '''
    rospy.Subscriber('change_slide', String, change_slide_callback)
    rospy.Subscriber('initiate_quiz', String, initiate_quiz_callback)
    rospy.Subscriber('toggle_control', String, toggle_control_callback)
    rospy.spin()


def start_global_publisher():
    '''
    Creates publishers which send commands to other modules.
    '''
    pub = rospy.Publisher('control', String, queue_size=10)
    rospy.loginfo("Global Publisher Node Started, now publishing messages")
    return pub



if __name__ == '__main__':
    try:
        rospy.init_node('Web_Publisher_Node_Local', xmlrpc_port=45100, tcpros_port=45101)
        start_global_listener()
    except Exception as e:
        print(e)