from .Publisher import *
from .Subscriber import *
from api.msg import CVInfo

# =========================================================

# Get information from other subteams
def Request(api_name, api_params={}):

	# Data class defined to store data from ROS Subscribers
	# in return value of Info.Request()
	class Data:
		SimpleMsg = ""
		Image = None
		LectureScript = ""
		Question = ""
		Answer = ""
		Joke = ""
		RaisedHandInfo = {}
		FaceInfo = {}
	
	# Callbacks for API
	if api_name == "SimpleMsg":
		"""Receive string message 
		@return	String
		"""
		def callback(msg):
			Data.SimpleMsg = msg.data
			rospy.loginfo("Received: SimpleMsg=%s" % Data.SimpleMsg)
		StringSubscriber("simple_msg", callback)
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
		ImageSubscriber("image", callback)
		return Data.Image
	

	## ========= WEB =========
	if api_name == "TriggerQuiz":
		"""Receive trigger_quiz signal"""
		callback = lambda _: rospy.loginfo("Received: trigger_quiz")
		StringSubscriber("trigger_quiz", callback)
		return True

	if api_name == "ChangeSlide":
		"""Receive change_slide signal"""
		callback = lambda _: rospy.loginfo("Received: change_slide")
		StringSubscriber("change_slide", callback)
		return True


	## ========= CV =========
	if api_name == "TriggerHandDetection":
		"""Receive trigger_hand_detection signal"""
		callback = lambda _: rospy.loginfo("Received: trigger_hand_detection")
		StringSubscriber("trigger_hand_detection", callback)
		return True

	if api_name == "TriggerAttentivenessDetection":
		"""Receive trigger_attentiveness_detection signal"""
		callback = lambda _: rospy.loginfo("Received: trigger_attentiveness_detection")
		StringSubscriber("trigger_attentiveness_detection", callback)
		return True


	## ========= NLP =========
	if api_name == "Question":
		"""Receive question STT from Speech"
		@return	question text
		"""
		def callback(msg):
			Data.Question = msg.data
			rospy.loginfo("Received: Question=%s" % Data.Question)
		StringSubscriber("question", callback)
		return Data.Question

	if api_name == "TriggerJoke":
		"""Receive trigger_joke signal"""
		callback = lambda _: rospy.loginfo("Received: trigger_joke")
		StringSubscriber("trigger_joke", callback)
		return True


	## ========= SPEECH =========
	if api_name == "LectureScript":
		"""Receive script text from NLP"
		@return	script text
		"""
		def callback(msg):
			Data.LectureScript = msg.data
			rospy.loginfo("Received: Script=%s" % Data.LectureScript)
		StringSubscriber("lecture_script", callback)
		return Data.LectureScript
	
	if api_name == "Answer":
		"""Receive answer text from NLP"
		@return	answer text
		"""
		def callback(msg):
			Data.Answer = msg.data
			rospy.loginfo("Received: Answer=%s" % Data.Answer)
		StringSubscriber("answer", callback)
		return Data.Answer

	if api_name == "Joke":
		"""Receive joke text from NLP"
		@return	joke text
		"""
		def callback(msg):
			Data.Joke = msg.data
			rospy.loginfo("Received: Joke=%s" % Data.Joke)
		StringSubscriber("joke", callback)
		return Data.Joke

	if api_name == "TriggerListen":
		"""Receive trigger_listen signal"""
		callback = lambda _: rospy.loginfo("Received: trigger_listen")
		StringSubscriber("trigger_listen", callback)
		return True

	if api_name == "TriggerNoiseDetection":
		"""Receive trigger_noise_detection signal"""
		callback = lambda _: rospy.loginfo("Received: trigger_noise_detection")
		StringSubscriber("trigger_noise_detection", callback)
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
			Data.RaisedHandInfo["bounding_box"] = (msg.x, msg.y, msg.w, msg.h)
			Data.RaisedHandInfo["frame_res"] = (msg.frame_width, msg.frame_height)
			Data.RaisedHandInfo["confidence_score"] = msg.score
			rospy.loginfo("Received: raised hand at (%.2f, %.2f)" % (msg.x, msg.y))
		CVInfoSubscriber("raised_hand", callback)
		return Data.RaisedHandInfo


	## ========= CONTROL =========
	if api_name == "IncrementLoopCounter":
		"""Receive increment_loop_counter signal"""
		callback = lambda _: rospy.loginfo("Received: increment_loop_counter")
		StringSubscriber("increment_loop_counter", callback)
		return True

	if api_name == "TakeControl":
		"""Receive take_control signal"""
		callback = lambda _: rospy.loginfo("Received: take_control")
		StringSubscriber("take_control", callback, listen_once=True)
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

	if api_name == "TriggerNoiseDetection":
		"""Send trigger_noise_detection to Speech"""
		trigger_noise_detection_publisher.publish(1)
		return True

	if api_name == "IncrementLoopCounter":
		"""Send increment_loop_counter to Control"""
		increment_loop_counter_publisher.publish(1)
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

	if api_name == "TriggerAttentivenessDetection":
		"""Send trigger_attentiveness_detection to CV"""
		trigger_attentiveness_detection_publisher.publish(1)
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
	if api_name == "ChangeSlide":
		"""Send change_slide to Speech"""
		change_slide_publisher.publish(1)
		return True

	
	## ========= SHARED =========
	if api_name == "TriggerQuiz":
		"""Send trigger_quiz to Web"""
		trigger_quiz_publisher.publish(1)
		return True

	if api_name == "TriggerJoke":
		"""Send trigger_joke to NLP"""
		trigger_joke_publisher.publish(1)
		return True


	print("API does not exist. Please check name again.")
	return False
