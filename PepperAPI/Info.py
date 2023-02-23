from PepperAPI import * # import global topic names
from .Publisher import *
from .Subscriber import *
from api.msg import CVInfo, State
import random
import rospy

# =========================================================

state_dict = {
	"Start": "",
	"AnyQuestions": "",
	"NoiseLevel": "",
	"Attentiveness": "",
	"NoQuestionsLoop": ""
}

# Get information from other subteams
def Request(api_name, api_params={}):

	# Data class defined to store data from ROS Subscribers
	# in return value of Info.Request()
	class Data:
		SimpleMsg = ""
		Image = None
		Slides = ""
		LectureScript = ""
		Question = ""
		Answer = ""
		Joke = ""
		Shutup = ""
		RaisedHandInfo = []
		FaceInfo = {}
		TriggerJokeOrQuiz = ""
		TriggerJokeOrShutup = ""
		NumHands = 0


	# Callbacks for API
	if api_name == "SimpleMsg":
		"""Receive string message 
		@return	String
		"""
		def callback(msg):
			Data.SimpleMsg = msg.data
			rospy.loginfo("Received: SimpleMsg=%s" % Data.SimpleMsg)
		StringSubscriber(SIMPLE_MSG_TOPIC, callback)
		return Data.SimpleMsg

	if api_name == "Image":
		"""Receive image
		@return	image
		"""
		def callback(msg):
			br = CvBridge()
			image = br.imgmsg_to_cv2(msg)
			Data.Image = image
			rospy.loginfo("Received: Image")
		ImageSubscriber(IMAGE_TOPIC, callback)
		return Data.Image
	

	## ========= WEB =========
	if api_name == "TriggerJokeOrQuiz":
		"""Receive trigger_quiz signal"""
		def callback(msg):
			Data.TriggerJokeOrQuiz = msg.data
			if Data.TriggerJokeOrQuiz == "joke":
				rospy.loginfo("Received: trigger_joke")
			elif Data.TriggerJokeOrQuiz == "quiz":
				rospy.loginfo("Received: trigger_quiz")
		StringSubscriber(TRIGGER_JOKE_OR_QUIZ_TOPIC, callback)
		return Data.TriggerJokeOrQuiz


	## ========= CV =========
	if api_name == "TriggerHandDetection":
		"""Receive trigger_hand_detection signal"""
		callback = lambda _: rospy.loginfo("Received: trigger_hand_detection")
		StringSubscriber(TRIGGER_HAND_DETECTION_TOPIC, callback)
		return True


	## ========= NLP =========
	if api_name == "Slides":
		"""Receive slides text from Web"
		@return	question text
		"""
		def callback(msg):
			Data.Slides = msg.data
			rospy.loginfo("Received: Slides=%s" % Data.Slides)
		StringSubscriber(SLIDES_TOPIC, callback)
		return Data.Slides

	if api_name == "Question":
		"""Receive question STT from Speech"
		@return	question text
		"""
		def callback(msg):
			Data.Question = msg.data
			rospy.loginfo("Received: Question=%s" % Data.Question)
		StringSubscriber(QUESTION_TOPIC, callback)
		return Data.Question

	# See api "TriggerJokeOrQuiz" under Web

	if api_name == "TriggerJokeOrShutup":
		"""Receive signal to trigger joke or shutup"""
		def callback(msg):
			Data.TriggerJokeOrShutup = msg.data
			if Data.TriggerJokeOrShutup == "joke":
				rospy.loginfo("Received: trigger_joke")
			elif Data.TriggerJokeOrShutup == "shutup":
				rospy.loginfo("Received: trigger_shutup")
		StringSubscriber(TRIGGER_JOKE_OR_SHUTUP_TOPIC, callback)
		return Data.TriggerJokeOrShutup

	## ========= SPEECH =========
	if api_name == "LectureScript":
		"""Receive script text from NLP"
		@return	script text
		"""
		def callback(msg):
			Data.LectureScript = msg.data
			rospy.loginfo("Received: Script=%s" % Data.LectureScript)
		StringSubscriber(LECTURE_SCRIPT_TOPIC, callback)
		return Data.LectureScript
	
	if api_name == "Answer":
		"""Receive answer text from NLP"
		@return	answer text
		"""
		def callback(msg):
			Data.Answer = msg.data
			rospy.loginfo("Received: Answer=%s" % Data.Answer)
		StringSubscriber(ANSWER_TOPIC, callback)
		return Data.Answer

	if api_name == "Joke":
		"""Receive joke text from NLP"
		@return	joke text
		"""
		def callback(msg):
			Data.Joke = msg.data
			rospy.loginfo("Received: Joke=%s" % Data.Joke)
		StringSubscriber(JOKE_TOPIC, callback)
		return Data.Joke

	if api_name == "Shutup":
		"""Receive shutup text from NLP"
		@return	shutup text : String
		"""
		def callback(msg):
			Data.Shutup = msg.data
			rospy.loginfo("Received: Shutup=%s" % Data.Shutup)
		StringSubscriber(SHUTUP_TOPIC, callback)
		return Data.Shutup

	if api_name == "TriggerListen":
		"""Receive trigger_listen signal"""
		callback = lambda _: rospy.loginfo("Received: trigger_listen")
		StringSubscriber(TRIGGER_LISTEN_TOPIC, callback)
		return True


	## ========= KINEMATICS =========
	if api_name == "RaisedHandInfo":
		"""Receive info on raised hand
		@return	dict{
			"bounding_box": (x,y,w,h)
			"frame_res": (width, height)
			"confidence_score": confidence_score
		}
		"""
		def callback(msg):
			Data.NumHands = int(msg.data)
		StringSubscriber(NUM_HANDS_TOPIC, callback)
		def callback(msg):
			Data.RaisedHandInfo.append(msg)
			# Data.RaisedHandInfo["bounding_box"] = (msg.x, msg.y, msg.w, msg.h)
			# Data.RaisedHandInfo["frame_res"] = (msg.frame_width, msg.frame_height)
			# Data.RaisedHandInfo["confidence_score"] = msg.score
			rospy.loginfo("Received: raised hand at (%.2f, %.2f)" % (msg.x, msg.y))
		CVInfoSubscriber(HAND_TOPIC, callback, listen=Data.NumHands)
		return Data.RaisedHandInfo


	## ========= CONTROL =========
	if api_name == "State":
		state_name = api_params["name"]
		def callback(msg):
			state = getattr(msg, state_name)
			Data.State = state if state else False
			rospy.loginfo("State: %s" % str(msg))
		StateSubscriber(STATE_TOPIC, callback)
		return Data.State
	
	if api_name == "TakeControl":
		"""Receive take_control signal"""
		callback = lambda _: rospy.loginfo("Received: take_control")
		StringSubscriber(TAKE_CONTROL_TOPIC, callback)
		return True


	print("API does not exist. Please check name again.")
	return False

# =========================================================

# Send information to other subteams
def Send(api_name, api_params={}):

	if api_name == "SimpleMsg":
		"""Send simple string message
		@param	api_params : dict{
			"value": message to be sent
		}
		"""
		msg = api_params["value"]
		simple_msg_publisher.publish(msg)
		return True

	if api_name == "Image":
		"""Send image
		@param	api_params : dict{
			"value": image
		}
		"""
		image = api_params["value"]
		image_publisher.publish(image)
		return True


	## ========= WEB =========
	if api_name == "Slides":
		"""Send slides text to NLP
		@param	api_params : dict{
			"text": text
		}
		"""
		text = api_params["text"]
		slides_publisher.publish(text)
		return True

	if api_name == "TakeControl":
		"""Send take_control to Control"""
		take_control_publisher.publish(1)
		return True


	## ========= CV =========
	if api_name == "NumHands":
		"""Send number of hands to Kinematics before sending RaisedHandInfo
		@param	api_params : dict{
			"value": number of hands
		}
		"""
		num_hands = api_params["value"]
		num_hands_publisher.publish(num_hands)
		return True

	if api_name == "RaisedHandInfo":
		"""Send information about raised hands
		@param	api_params : dict{
			"bounding_box": (x,y,w,h)
			"frame_res": (width, height)
			"confidence_score": confidence score
		}
		"""
		bounding_box = api_params["bounding_box"]
		frame_res = api_params["frame_res"]
		confidence_score = api_params["confidence_score"]
		number = api_params["number"]
		hand_publisher.publish(bounding_box, frame_res, confidence_score, number)
		return True

	if api_name == "FaceInfo":
		"""Send information about raised hands
		@param	api_params : dict{
			"bounding_box": (x,y,w,h)
			"frame_res": (width, height)
			"engagement_score": engagement score
		}
		"""
		bounding_box = api_params["bounding_box"]
		frame_res = api_params["frame_res"]
		engagement_score = api_params["engagement_score"]
		face_publisher.publish(bounding_box, frame_res, engagement_score)
		return True


	## ========= NLP =========
	if api_name == "LectureScript":
		"""Send lecture script to Speech
		@param	api_params : dict{
			"text": text
		}
		"""
		text = api_params["text"]
		lecture_script_publisher.publish(text)
		return True

	if api_name == "Answer":
		"""Send QnA answer to Speech
		@param	api_params : dict{
			"text": text
		}
		"""
		text = api_params["text"]
		answer_publisher.publish(text)
		return True

	if api_name == "Joke":
		"""Send joke text to Speech
		@param	api_params : dict{
			"text": text
		}
		"""
		text = api_params["text"]
		joke_publisher.publish(text)
		return True

	if api_name == "Shutup":
		"""Send shutup text to Speech
		@param	api_params : dict{
			"text": text
		}
		"""
		text = api_params["text"]
		shutup_publisher.publish(text)
		return True


	## ========= SPEECH =========
	if api_name == "Question":
		"""Send QnA question to NLP
		@param	api_params : dict{
			"text": text
		}
		"""
		text = api_params["text"]
		question_publisher.publish(question)
		return True


	## ========= KINEMATICS =========
	if api_name == "TriggerHandDetection":
		"""Send trigger_hand_detection to CV"""
		trigger_hand_detection_publisher.publish(1)
		return True

	if api_name == "TriggerListen":
		"""Send trigger_listen to Speech"""
		trigger_listen_publisher.publish(1)
		return True


	## ========= CONTROL =========
	if api_name == "TriggerJokeOrQuiz":
		"""Randomly choose between sending trigger_quiz to Web or trigger_joke to NLP"""
		choice = random.choice(["joke","quiz"])
		trigger_joke_quiz_publisher.publish(choice)
		return choice

	if api_name == "TriggerJokeOrShutup":
		"""Randomly choose between sending trigger_shutup or trigger_joke to NLP"""
		choice = random.choice(["joke","shutup"])
		trigger_joke_shutup_publisher.publish(choice)
		return choice

	if api_name == "State":
		"""Update state"""
		global state_dict
		for k,v in api_params.items():
			state_dict[k] = v
		# state_publisher.publish(state_dict)
		return True


	print("API does not exist. Please check name again.")
	return False


# Only for Control. Broadcast states to all modules
def Broadcast(api_name):

	if api_name == "State":
		rate = rospy.Rate(1)
		while True:
			global state_dict
			state_publisher.publish(state_dict)
			rate.sleep()
		
	return
