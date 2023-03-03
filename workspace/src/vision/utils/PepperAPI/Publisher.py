import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from api.msg import CVInfo
from cv_bridge import CvBridge

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
		rospy.loginfo("Published image")

# ==============================================================
# Initiate publishers
simple_msg_publisher = StringPublisher("simple_msg")
image_publisher = ImagePublisher("image")
tts_publisher = StringPublisher("tts")

## WEB
slides_publisher = StringPublisher("slides")

## CV
hands_publisher = CVInfoPublisher("raised_hands")
face_publisher = CVInfoPublisher("faces")
trigger_noise_detection_publisher = StringPublisher("trigger_noise_detection")
increment_loop_counter_publisher = StringPublisher("increment_loop_counter")

## NLP
lecture_script_publisher = StringPublisher("lecture_script")
answer_publisher = StringPublisher("answer")
joke_publisher = StringPublisher("joke")

## SPEECH
question_publisher = StringPublisher("question")
audio_player_publisher = StringPublisher("audio_player_topic")
trigger_attentiveness_detection_publisher = StringPublisher("trigger_attentiveness_detection")

## KINEMATICS
trigger_hand_detection_publisher = StringPublisher("trigger_hand_detection")
trigger_listen_publisher = StringPublisher("trigger_listen")

## CONTROL
change_slide_publisher = StringPublisher("change_slide")

## SHARED
trigger_quiz_publisher = StringPublisher("trigger_quiz")
trigger_joke_publisher = StringPublisher("trigger_joke")
