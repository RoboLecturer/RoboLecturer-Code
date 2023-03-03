import rospy
import rosnode

# Globals
ROBOT_IP = "192.168.0.102"
ROBOT_PORT = 9559
PEPPER_USER = "nao"
PEPPER_PASSWORD = "BioARTLab123"
PEPPER_AUDIO_PATH = "/home/nao/audio_files/"

def init(node):
	# XMLRPC & TCPROS ports to start from 45100 & 45101
	_xmlrpc, _tcpros = 45100, 45101

	# Since every script calling the API requires a separate node,
	# check for nodes with the same name and append an incrementing counter
	# to different nodes from different scripts
	nodes = rosnode.get_node_names()
	num = 0
	node_name = "%s_%d" % (node, num)
	while "/"+node_name in nodes:
		num += 1
		_xmlrpc += 2
		_tcpros += 2
		node_name = "%s_%d" % (node, num)

	# Initialize the node
	rospy.init_node(node_name, xmlrpc_port=_xmlrpc, tcpros_port=_tcpros, disable_signals=True)
	rospy.loginfo("Node initialised: " + node_name)
