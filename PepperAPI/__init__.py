import rospy
import rosnode
import os

# set to True if you're doing self testing
TEST_DUMMY = True

# Globals
ROBOT_IP = "192.168.0.100" if "NAO_IP" not in os.environ else os.environ["NAO_IP"]
ROBOT_PORT = 9559
PEPPER_USER = "nao"
PEPPER_PASSWORD = "BioARTLab123"
PEPPER_AUDIO_PATH = "/home/nao/audio_files/"

# Topics
ANSWER_TOPIC = "answer"
AUDIO_PLAYER_TOPIC = "audio_player"
CHANGE_SLIDE_TOPIC = "change_slide"
FACE_TOPIC = "face"
HAND_TOPIC = "raised_hand"
IMAGE_TOPIC = "image"
JOKE_TOPIC = "joke"
LECTURE_SCRIPT_TOPIC = "lecture_script"
NUM_HANDS_TOPIC = "num_hands"
NUM_SCRIPTS_TOPIC = "num_scripts"
NUM_SLIDES_TOPIC = "num_slides"
POINT_TOPIC = "point"
QUESTION_TOPIC = "question"
SHUTUP_TOPIC = "shutup"
SIMPLE_MSG_TOPIC = "simple_msg"
SLIDES_TOPIC = "slides"
STATE_TOPIC = {
	"Start": "state_start",
	"AnyQuestions": "state_any_questions",
	"NoiseLevel": "state_noise_level",
	"Attentiveness": "state_attentiveness",
	"NoQuestionsLoop": "state_no_questions"
}
STATE_UPDATE_TOPIC = "state_update"
STUDENT_DONE_TOPIC = "student_done"
TAKE_CONTROL_TOPIC = "take_control"
TRIGGER_HAND_DETECTION_TOPIC = "trigger_hand_detection"
TRIGGER_JOKE_OR_SHUTUP_TOPIC = "trigger_joke_or_shutup"
TRIGGER_JOKE_OR_QUIZ_TOPIC = "trigger_joke_or_quiz"
TRIGGER_LISTEN_TOPIC = "trigger_listen"
TRIGGER_QUIZ_TOPIC = "trigger_quiz"
TTS_TOPIC = "tts"
VOLUME_TOPIC = "volume"

# Start node
def init(node):
	global node_initiated

	# If doing dummy testing on one machine, node names must be the same
	if TEST_DUMMY:
		node = "master"

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
