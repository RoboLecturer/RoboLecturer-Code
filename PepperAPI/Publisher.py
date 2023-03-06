import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from api.msg import CVInfo
# from cv_bridge import CvBridge
from PepperAPI import * # import global topic names

# Parent publisher class
class Publisher:
	def __init__(self, name, topic, payload_type, num_subscribers=1, log=False):
		self.name = name
		self.topic = topic
		self.publisher = rospy.Publisher(self.topic, payload_type, queue_size=10)
		self.num_subscribers = num_subscribers
		if log:
			self.log()

	# Log info about publisher
	def log(self):
		print("[%s] topic=%s" % (self.name, self.topic))

# Child publisher classes
"""publish string msg"""
class StringPublisher(Publisher, object):
	def __init__(self, topic, num_subscribers=1, log=False):
		super(StringPublisher, self).__init__(
			"StringPublisher",
			topic, 
			String,
			num_subscribers,
			log)

	def publish(self, msg):
		msg = str(msg)
		while self.publisher.get_num_connections() < self.num_subscribers:
			pass			
		self.publisher.publish(msg)
		if self.topic not in [CHANGE_SLIDE_TOPIC, TRIGGER_QUIZ_TOPIC]:
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
		while self.publisher.get_num_connections() < self.num_subscribers:
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
		while self.publisher.get_num_connections() < self.num_subscribers:
			pass			
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
		while self.publisher.get_num_connections() < self.num_subscribers:
			pass			
		self.publisher.publish(br.cv2_to_imgmsg(image))
		rospy.loginfo("[%s] %s: Published image" % (self.name, self.topic))


# ==============================================================
# Initiate publishers
simple_msg_publisher = StringPublisher(SIMPLE_MSG_TOPIC)
image_publisher = ImagePublisher(IMAGE_TOPIC)
tts_publisher = StringPublisher(TTS_TOPIC)

## WEB
slides_publisher = StringPublisher(SLIDES_TOPIC)
take_control_publisher = StringPublisher(TAKE_CONTROL_TOPIC)
num_slides_publisher = StringPublisher(NUM_SLIDES_TOPIC)

## CV
hand_publisher = CVInfoPublisher(HAND_TOPIC)
num_hands_publisher = StringPublisher(NUM_HANDS_TOPIC)
face_publisher = CVInfoPublisher(FACE_TOPIC)

## NLP
lecture_script_publisher = StringPublisher(LECTURE_SCRIPT_TOPIC)
answer_publisher = StringPublisher(ANSWER_TOPIC)
joke_publisher = StringPublisher(JOKE_TOPIC)
shutup_publisher = StringPublisher(SHUTUP_TOPIC)
<<<<<<< HEAD
=======
num_scripts_publisher = StringPublisher(NUM_SCRIPTS_TOPIC)
>>>>>>> 4153c11c353cf1aeebfee808772e2a966f6db8b7

## SPEECH
question_publisher = StringPublisher(QUESTION_TOPIC)
audio_player_publisher = StringPublisher(AUDIO_PLAYER_TOPIC)

## KINEMATICS
trigger_hand_detection_publisher = StringPublisher(TRIGGER_HAND_DETECTION_TOPIC)
trigger_listen_publisher = StringPublisher(TRIGGER_LISTEN_TOPIC)
point_publisher = CVInfoMsgPublisher(POINT_TOPIC)
volume_publisher = StringPublisher(VOLUME_TOPIC)

## CONTROL
state_publisher = { 
	state: StringPublisher(STATE_TOPIC[state], num_subscribers=3)
	for state in STATE_TOPIC 
}
state_update_publisher = StringPublisher(STATE_UPDATE_TOPIC)
change_slide_publisher = StringPublisher(CHANGE_SLIDE_TOPIC, num_subscribers=0)
trigger_quiz_publisher = StringPublisher(TRIGGER_QUIZ_TOPIC, num_subscribers=0)

## SHARED
trigger_joke_or_shutup_publisher = StringPublisher(TRIGGER_JOKE_OR_SHUTUP_TOPIC, num_subscribers=2)
trigger_joke_or_quiz_publisher = StringPublisher(TRIGGER_JOKE_OR_QUIZ_TOPIC, num_subscribers=2)
