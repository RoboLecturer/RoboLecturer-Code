from PepperAPI import * # import global topic names
from .Publisher import *
from .Subscriber import *
from api.msg import CVInfo
import random, rospy, threading

# =========================================================
# Get information from other subteams

def Request(api_name, api_params={}):
	
	# Data class defined to store data from ROS Subscribers
	# in return value of Info.Request()
	class Data:
		SimpleMsg = ""
		Image = None
		Slides = []
		NumSlides = 0
		LectureScript = ""
		Question = ""
		Answer = ""
		Joke = ""
		Shutup = ""
		NumHands = 0
		RaisedHandInfo = []
		TriggerJokeOrQuiz = ""
		TriggerJokeOrShutup = ""
		State = ""
		ChangeSlide = ""


	# API callbacks
	if api_name == "SimpleMsg":
		"""Receive string message 
		@return	string message : String
		"""
		def callback(msg):
			Data.SimpleMsg = msg.data
			rospy.loginfo("Received: SimpleMsg=%s" % Data.SimpleMsg)
		StringSubscriber(SIMPLE_MSG_TOPIC, callback)
		return Data.SimpleMsg

	if api_name == "Image":
		"""Receive image
		@return	image : numpy.ndarray
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
		"""Receive trigger_quiz signal
		@return triggered signal (joke/quiz) : String
		"""
		def callback(msg):
			Data.TriggerJokeOrQuiz = msg.data
			if Data.TriggerJokeOrQuiz == "joke":
				rospy.loginfo("Received: trigger_joke")
			elif Data.TriggerJokeOrQuiz == "quiz":
				rospy.loginfo("Received: trigger_quiz")
		StringSubscriber(TRIGGER_JOKE_OR_QUIZ_TOPIC, callback)
		return Data.TriggerJokeOrQuiz

	if api_name == "ChangeSlide":
		"""Receive command to change_slide"""
		def callback(msg):
			Data.ChangeSlide = msg.data
			rospy.loginfo("Received: change_slide=%s" % Data.ChangeSlide)
		StringSubscriber(CHANGE_SLIDE_TOPIC, callback)
		return Data.ChangeSlide


	## ========= CV =========
	if api_name == "TriggerHandDetection":
		"""Receive trigger_hand_detection signal"""
		callback = lambda _: rospy.loginfo("Received: trigger_hand_detection")
		StringSubscriber(TRIGGER_HAND_DETECTION_TOPIC, callback)
		return True


	## ========= NLP =========
	if api_name == "Slides":
		"""Receive slides text from Web"
		@return	slides text : String
		"""
		def callback(msg):
			Data.NumSlides = int(msg.data)
		StringSubscriber(NUM_SLIDES_TOPIC, callback)
		def callback(msg):
			Data.Slides.append(msg)
			rospy.loginfo("Received: Slides=%s" % Data.Slides)
		StringSubscriber(SLIDES_TOPIC, callback, listen=Data.NumSlides)
		return Data.Slides

	if api_name == "Question":
		"""Receive question STT from Speech"
		@return	question text : String
		"""
		def callback(msg):
			Data.Question = msg.data
			rospy.loginfo("Received: Question=%s" % Data.Question)
		StringSubscriber(QUESTION_TOPIC, callback)
		return Data.Question

	# See api "TriggerJokeOrQuiz" under Web

	if api_name == "TriggerJokeOrShutup":
		"""Receive signal to trigger joke or shutup
		@return triggered signal (joke/shutup) : String
		"""
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
		@return	script text : String
		"""
		def callback(msg):
			Data.LectureScript = msg.data
			rospy.loginfo("Received: Script=%s" % Data.LectureScript)
		StringSubscriber(LECTURE_SCRIPT_TOPIC, callback)
		return Data.LectureScript
	
	if api_name == "Answer":
		"""Receive answer text from NLP"
		@return	answer text : String
		"""
		def callback(msg):
			Data.Answer = msg.data
			rospy.loginfo("Received: Answer=%s" % Data.Answer)
		StringSubscriber(ANSWER_TOPIC, callback)
		return Data.Answer

	if api_name == "Joke":
		"""Receive joke text from NLP"
		@return	joke text : String
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
		@return	list of CVInfo msgs of raised hand info
		"""
		def callback(msg):
			Data.NumHands = int(msg.data)
			rospy.loginfo("Number of hands detected: %d" % Data.NumHands)
		StringSubscriber(NUM_HANDS_TOPIC, callback)
		def callback(msg):
			Data.RaisedHandInfo.append(msg)
			rospy.loginfo("Received: raised hand at (%.2f, %.2f)" % (msg.x, msg.y))
		CVInfoSubscriber(HAND_TOPIC, callback, listen=Data.NumHands)
		return Data.RaisedHandInfo


	## ========= CONTROL =========
	if api_name == "TakeControl":
		"""Receive take_control signal"""
		callback = lambda _: rospy.loginfo("Received: take_control")
		StringSubscriber(TAKE_CONTROL_TOPIC, callback)
		return True


	## ========= SHARED =========
	if api_name == "State":
		"""Receive state
		@param	api_params : dict{
			"name": (String) name of state to be queried
			"print": False if headers shouldn't be printed
		}
		"""
		name = api_params["name"]
		def callback(msg):
			Data.State = msg.data
			if "print" not in api_params:
				print("\n========= STATE: %s =========" % name)
			rospy.loginfo("Received: State %s=%s" % (name, Data.State))
		StringSubscriber(STATE_TOPIC[name], callback, log=False)
		return Data.State


	print("Info.Request(%s) does not exist. Please check name again." % api_name)
	return


# =========================================================
# Send information to other subteams

def Send(api_name, api_params={}):

	# API callbacks
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

	if api_name == "NumSlides":
		"""Send number of hands to Kinematics before sending RaisedHandInfo
		@param	api_params : dict{
			"value": number of slides
		}
		"""
		num_slides = api_params["value"]
		num_slides_publisher.publish(num_slides)
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
		hand_publisher.publish(bounding_box, frame_res, confidence_score)
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
		question_publisher.publish(text)
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
		if "force" in api_params:
			choice = api_params["force"]
		trigger_joke_or_quiz_publisher.publish(choice)
		return choice

	if api_name == "TriggerJokeOrShutup":
		"""Randomly choose between sending trigger_shutup or trigger_joke to NLP"""
		choice = random.choice(["joke","shutup"])
		if "force" in api_params:
			choice = api_params["force"]
		trigger_joke_or_shutup_publisher.publish(choice)
		return choice

	if api_name == "ChangeSlide":
		"""Send command to Web to change slide
		@params	api_params : dict{
			"cmd": "increment|", "decrement|" or "goto|<slide_num>"
		}
		"""
		cmd = api_params["cmd"]
		change_slide_publisher.publish(cmd)
		return True


	## ========= SHARED =========
	if api_name == "State":
		"""Update state
		@param	api_params : dict{
			(String) state name : (String) new value,
			"print": False if headers shouldn't be printed
		"""
		name = [k for k in api_params if k != "print"][0]
		value = api_params[name]
		msg = "%s=%s" % (name, value)
		if value and "print" not in api_params:
			print("\n========= STATE: %s =========" % name)
		state_update_publisher.publish(msg)
		return True


	print("Info.Send(%s) does not exist. Please check name again." % api_name)
	return


# =========================================================
# Only to be used by Control module

def Listen():

	"""Listen to state change and update accordingly"""		
	def callback(msg):
		[name, value] = msg.data.split("=")
		if value:
			state_publisher[name].publish(value)
			rospy.loginfo("Received: Update %s=%s" % (name,value))

	def subscribe_listen():
		StringSubscriber(STATE_UPDATE_TOPIC, callback, listen=0)
		while event.is_set() and not rospy.is_shutdown():
			rospy.rostime.wallsleep(0.5)

	event = threading.Event()
	event.set()
	t = threading.Thread(target=subscribe_listen)
	t.start()

	# Exit when KeyboardInterrupt or when killed
	global kill_listen
	try:
		while True:
			if not kill_listen:
				continue
			event.clear()
			t.join()
			break
	except KeyboardInterrupt:
		print("KeyboardInterrupt")
		event.clear()
		t.join()

	return

# Kill Info.Listen()
kill_listen = False
def StopListen():
	global kill_listen
	kill_listen = True
