import rospy
import rosnode

# Globals
ROBOT_IP = "192.168.0.100"
ROBOT_PORT = 9559
PEPPER_USER = "nao"
PEPPER_PASSWORD = "BioARTLab123"
PEPPER_AUDIO_PATH = "/home/nao/audio_files/"

# Topics
SIMPLE_MSG_TOPIC = "simple_msg"
IMAGE_TOPIC = "image"
TTS_TOPIC = "tts"
SLIDES_TOPIC = "slides"
NUM_HANDS_TOPIC = "num_hands"
HAND_TOPIC = "raised_hand"
FACE_TOPIC = "face"
LECTURE_SCRIPT_TOPIC = "lecture_script"
QUESTION_TOPIC = "question"
ANSWER_TOPIC = "answer"
JOKE_TOPIC = "joke"
SHUTUP_TOPIC = "shutup"
AUDIO_PLAYER_TOPIC = "audio_player"
STATE_TOPIC = "state"
POINT_TOPIC = "point"
TAKE_CONTROL_TOPIC = "take_control"
TRIGGER_HAND_DETECTION_TOPIC = "trigger_hand_detection"
TRIGGER_LISTEN_TOPIC = "trigger_listen"
TRIGGER_JOKE_OR_QUIZ_TOPIC = "trigger_joke_or_quiz"
TRIGGER_JOKE_OR_SHUTUP_TOPIC = "trigger_joke_or_shutup"

# Start node
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
