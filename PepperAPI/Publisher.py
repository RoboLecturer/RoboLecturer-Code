import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from api.msg import CVInfo, State
from cv_bridge import CvBridge
from PepperAPI import * # import global topic names

# Parent publisher class
class Publisher:
	def __init__(self, name, topic, payload_type):
		self.name = name
		self.topic = topic
		self.publisher = rospy.Publisher(self.topic, payload_type, queue_size=10)
		# self.log()

	# Log info about publisher
	def log(self):
		print("[%s] topic=%s" % (self.name, self.topic))

# Child publisher classes
"""publish string msg"""
class StringPublisher(Publisher, object):
	def __init__(self, topic):
		super(StringPublisher, self).__init__(
			"StringPublisher",
			topic, 
			String)

	def publish(self, msg):
		msg = str(msg)
		while self.publisher.get_num_connections() == 0:
			pass			
		self.publisher.publish(msg)
		rospy.loginfo("[%s] %s: %s" % (self.name, self.topic, msg))

"""publish array of xy float coordinates"""
class CVInfoPublisher(Publisher, object):
	def __init__(self, topic):
		super(CVInfoPublisher, self).__init__(
			"CVInfoPublisher",
			topic,
			CVInfo)

	def publish(self, bounding_box, frame_res, confidence_score):
		msg = CVInfo()
		msg.x, msg.y, msg.w, msg.h = bounding_box
		msg.frame_width = frame_res[0]
		msg.frame_height = frame_res[1]
		msg.score = confidence_score
		while self.publisher.get_num_connections() == 0:
			pass			
		self.publisher.publish(msg)
		rospy.loginfo("[%s] %s: Published hand info (%.0f, %.0f)" % (self.name, self.topic, msg.x, msg.y))

"""publish msg"""
class CVInfoMsgPublisher(Publisher, object):
	def __init__(self, topic):
		super(CVInfoMsgPublisher, self).__init__(
			"CVInfoMsgPublisher",
			topic,
			CVInfo)

	def publish(self, msg):
		self.publisher.publish(msg)
		rospy.loginfo("[%s] %s: Published hand info" % (self.name, self.topic))


"""publish image"""
class ImagePublisher(Publisher, object):
	def __init__(self, topic):
		super(ImagePublisher, self).__init__(
			"ImagePublisher",
			topic,
			Image)

	def publish(self, image):
		br = CvBridge()
		while self.publisher.get_num_connections() == 0:
			pass			
		self.publisher.publish(br.cv2_to_imgmsg(image))
		rospy.loginfo("[%s] %s: Published image" % (self.name, self.topic))

"""publish state info"""
class StatePublisher(Publisher, object):
	def __init__(self, topic, wait=True):
		super(StatePublisher, self).__init__(
			"StatePublisher",
			topic,
			State)
		self.wait = wait

	def publish(self, state_dict):
		msg = State()
		msg.Start = state_dict["Start"]
		msg.AnyQuestions = state_dict["AnyQuestions"]
		msg.NoiseLevel = state_dict["NoiseLevel"]
		msg.Attentiveness = state_dict["Attentiveness"]
		msg.NoQuestionsLoop = state_dict["NoQuestionsLoop"]
		if self.wait:
			while self.publisher.get_num_connections() == 0:			
				pass			
		self.publisher.publish(msg)


# ==============================================================
# Initiate publishers
simple_msg_publisher = StringPublisher(SIMPLE_MSG_TOPIC)
image_publisher = ImagePublisher(IMAGE_TOPIC)
tts_publisher = StringPublisher(TTS_TOPIC)

# VOLUME UP/DOWN
volume_up_publisher = StringPublisher(VOLUME_UP_TOPIC)
volume_down_publisher = StringPublisher(VOLUME_DOWN_TOPIC)

## WEB
slides_publisher = StringPublisher(SLIDES_TOPIC)
take_control_publisher = StringPublisher(TAKE_CONTROL_TOPIC)

## CV
hand_publisher = CVInfoPublisher(HAND_TOPIC)
num_hands_publisher = StringPublisher(NUM_HANDS_TOPIC)
face_publisher = CVInfoPublisher(FACE_TOPIC)

## NLP
lecture_script_publisher = StringPublisher(LECTURE_SCRIPT_TOPIC)
answer_publisher = StringPublisher(ANSWER_TOPIC)
joke_publisher = StringPublisher(JOKE_TOPIC)
shutup_publisher = StringPublisher(SHUTUP_TOPIC)

## SPEECH
question_publisher = StringPublisher(QUESTION_TOPIC)
audio_player_publisher = StringPublisher(AUDIO_PLAYER_TOPIC)

## KINEMATICS
trigger_hand_detection_publisher = StringPublisher(TRIGGER_HAND_DETECTION_TOPIC)
trigger_listen_publisher = StringPublisher(TRIGGER_LISTEN_TOPIC)
point_publisher = CVInfoMsgPublisher(POINT_TOPIC)

## CONTROL
# state_publisher to keep publishing even without active subscribers
state_publisher = StatePublisher(STATE_TOPIC, wait=False)
state_update_publisher = StatePublisher(STATE_UPDATE_TOPIC)

## SHARED
trigger_joke_or_quiz_publisher = StringPublisher(TRIGGER_JOKE_OR_QUIZ_TOPIC)
trigger_joke_or_shutup_publisher = StringPublisher(TRIGGER_JOKE_OR_SHUTUP_TOPIC)
