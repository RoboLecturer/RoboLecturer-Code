from __future__ import print_function
import sys
import time
import threading
import signal
import rospy
import paramiko
import json
from .Publisher import *
from .Subscriber import *
from PepperAPI import *

# =========================================================
# Request for actuator action by Pepper robot
# Used by subteams

def Request(api_name, api_params={}):

	class FastTransport(paramiko.Transport):
		def __init__(self, sock):
			super(FastTransport, self).__init__(sock)
			self.window_size = 2147483647
			self.packetizer.REKEY_BYTES = pow(2, 40)
			self.packetizer.REKEY_PACKETS = pow(2, 40)
			self.use_compression()

	# API callbacks
	if api_name == "ALTextToSpeech":
		"""Send message to be said by Pepper
		@param api_params : dict{
			"value": (String) message to be sent
		}
		"""
		msg = api_params["value"]
		tts_publisher.publish(msg)
		return True
	
	if api_name == "ALAudioPlayer":
		"""Request for audio file to be played through Pepper's speakers
		@param api_params : dict{
			"path": (String) path of audio file in your machine
			"file": (String) name of file that's already in Pepper (skips uploading)
			"length": (Float) length of audio file in seconds
		}
		"""
		def printProgress(transferred, left):
			print("Transferred: {0}\tOut of: {1}".format(transferred, left), end="\r")

		if "file" not in api_params:
			# Define paths
			soundfile_path = api_params["path"]
			filename = soundfile_path.split("/")[-1]
			pepper_path = PEPPER_AUDIO_PATH + filename

			if not TEST_DUMMY:
				# Setup SFTP link
				transport = FastTransport((ROBOT_IP, 22))
				transport.connect(username=PEPPER_USER, password=PEPPER_PASSWORD)
				sftp = paramiko.SFTPClient.from_transport(transport)

				# Send file
				sftp.put(soundfile_path, pepper_path, callback=printProgress)
				sftp.close()
				transport.close()

		else:
			filename = api_params["file"]

		duration = -1.0 if "length" not in api_params else api_params["length"]
		data = {
			"file": filename,
			"length": duration
		}
		data_to_string = json.dumps(data)

		# Publish msg to kinematics module to play audio
		audio_player_publisher.publish(data_to_string)
		return True

	if api_name == "Point":
		"""Request for Pepper to point to raised hand at provided coords
		@param api_params : dict{
			"info": CVInfo msg of hand info
		}
		"""
		msg = api_params["info"]
		point_publisher.publish(msg)
		return True

	if api_name == "ChangeVolume":
		"""Request for Pepper to point to raised hand at provided coords
		@param api_params : dict{
			"cmd": (String) "up" or "down"
		}
		"""
		val = api_params["cmd"]
		volume_publisher.publish(val)
		return True

	
	print("Action.Request(%s) does not exist. Please check name again." % api_name)
	return


# ==============================================================
# Only to be used by Kinematics module
# Listen to requests for Pepper controls

D = 0 # Pepper's offset from the world (CV camera) in px
def Listen():

	# Import NAOqi modules
	if not TEST_DUMMY:
		from naoqi import ALProxy, ALBroker
		from .motions import MOTIONS, ResetPosition, PointTimeline
		import almath

	# Callback for ALTextToSpeech
	def tts_callback(msg):
		tts = ALProxy("ALTextToSpeech", ROBOT_IP, ROBOT_PORT)
		rospy.loginfo("Pepper ALTextToSpeech: Say %s" % msg.data)
		if not TEST_DUMMY:
			tts.say(msg.data)
		return IsDone("Set", "ALTextToSpeech")

	# Callback for ALAudioPlayer
	def audio_player_callback(msg):

		data = json.loads(msg.data)
		filename = str(data["file"])
		length = float(data["length"])
		audio_file = PEPPER_AUDIO_PATH + filename

		# Play audio in separate thread
		rospy.loginfo("Pepper ALAudioPlayer: Play audio %s (%.3fs)" % (audio_file, length))

		if not TEST_DUMMY:
			ap = ALProxy("ALAudioPlayer", ROBOT_IP, ROBOT_PORT)
			taskId = ap.post.playFile(audio_file)

			# For files pre-uploaded in Pepper, length is not given,
			# and can be directly gotten because they're WAV files
			if length < 0:
				length = ap.getFileLength(taskId)

			actions_list = []
			motion = None
			if length > 10:
				playing_length = length
				while playing_length > 2:
					i = np.random.randint(len(MOTIONS))
					action = MOTIONS.keys()[i]
					action_time = MOTIONS[action]
					playing_length -= action_time + 0.5
					actions_list.append(action)
				motion = ALProxy("ALMotion", ROBOT_IP, ROBOT_PORT)

			# Delay while file is being played in thread
			global interrupt # for interrupting playback by sending SIGUSR1
			global D, LOCALIZING
			tic = time.time()
			toc = time.time()
			action_counter = 0
			while (toc - tic) < length and not interrupt:
				if len(actions_list) and action_counter < len(actions_list):
					action = actions_list[action_counter]
					names, times, keys = action()
					times = [[k * SLOW_DOWN for k in t] for t in times]
					try:
						if LOCALIZING and D != 0:
							chance = np.random.random()
							if chance > 0.96:
								direction = -1 if D < 0 else 1
								motion.setExternalCollisionProtectionEnabled("All", False)
								motion.move(0, 0.15*direction, 0)
								time.sleep(np.random.random() * 1.5)
						motion.move(0,0,0)
						motion.setExternalCollisionProtectionEnabled("All", True)
						motion.angleInterpolationBezier(names, times, keys)
						time.sleep(0.2)
						action_counter += 1
					except RuntimeError:
						motion = ALProxy("ALMotion", ROBOT_IP, ROBOT_PORT)
				toc = time.time()


			# Stop playback
			try:
				ap.stopAll()
			except:
				pass
			
			motion = ALProxy("ALMotion", ROBOT_IP, ROBOT_PORT)
			names, times, keys = ResetPosition()
			motion.angleInterpolationBezier(names, times, keys)

		else:
			time.sleep(2) # slight buffer

		interrupt = False # reset interrupt

		return IsDone("Set", "ALAudioPlayer")

	# Callback for pointing at raised hand
	def point_callback(msg):
		# parse msg
		x,y,w,h = msg.x, msg.y, msg.w, msg.h
		w,h = 0,0
		frame_width = msg.frame_width
		frame_height = msg.frame_height
		
		global D

		# constants
		HEAD_PITCH_UP = -5.0
		HEAD_PITCH_DOWN = 10.0
		HEAD_YAW_IN = 0.0
		HEAD_YAW_OUT = 45.0 # left
		SHOULDER_PITCH_UP = 15.0
		SHOULDER_PITCH_DOWN = 35.0
		LSHOULDER_ROLL_OUT = 60.0
		LSHOULDER_ROLL_IN = 5.0
		RSHOULDER_ROLL_OUT = -LSHOULDER_ROLL_OUT
		RSHOULDER_ROLL_IN = -LSHOULDER_ROLL_IN
	
		# parameters
		center_x = x + w//2
		center_y = y + h//2
		mid_width = frame_width // 2
		rospy.loginfo("Raised hand at (%.0f, %.0f) of (%d, %d)" %
			(center_x, center_y, frame_width, frame_height))

		# extension range for pointing based on x-offset from center
		ROLL_EXT = LSHOULDER_ROLL_OUT * D / (mid_width)
		YAW_EXT = HEAD_YAW_OUT * D / (mid_width)
		
		# point left/right
		if center_x < mid_width + D:
			effector = "L"
			shoulder_roll = (LSHOULDER_ROLL_OUT + ROLL_EXT) - (LSHOULDER_ROLL_OUT + ROLL_EXT - LSHOULDER_ROLL_IN) / (mid_width + D) * center_x
			head_yaw = (HEAD_YAW_OUT + YAW_EXT) - (HEAD_YAW_OUT + YAW_EXT) / (mid_width + D) * center_x
		else:
			effector = "R"
			shoulder_roll = RSHOULDER_ROLL_IN - (RSHOULDER_ROLL_IN - (RSHOULDER_ROLL_OUT + ROLL_EXT)) / (mid_width - D) * (center_x - mid_width - D)
			head_yaw = (YAW_EXT - HEAD_YAW_OUT) / (mid_width - D) * (center_x - mid_width - D)

		# point up/down
		shoulder_pitch = SHOULDER_PITCH_UP + (SHOULDER_PITCH_DOWN - SHOULDER_PITCH_UP) / frame_height * center_y
		head_pitch = HEAD_PITCH_UP + (HEAD_PITCH_DOWN - HEAD_PITCH_UP) / frame_height * center_y

		# actuate
		rospy.loginfo("Pepper ALTracker: Point %s at with head_pitch=%.2f, head_yaw=%.2f, shoulder_pitch=%.2f, shoulder_roll=%.2f with offset %d from world center" %
			(effector+"Arm", head_pitch, head_yaw, shoulder_pitch, shoulder_roll, D))

		if not TEST_DUMMY:
			motion = ALProxy("ALMotion", ROBOT_IP, ROBOT_PORT)
			names, times, keys = PointTimeline(effector, head_pitch, head_yaw, shoulder_pitch, shoulder_roll)
			motion.post.angleInterpolationBezier(names, times, keys)
			time.sleep(0.7)

		return IsDone("Set", "Point")

	# Increase/decrease master volume
	def volume_callback(msg):
		rospy.loginfo("Pepper ALAudioDevice: Volume " + msg.data)
		if not TEST_DUMMY:
			ad = ALProxy("ALAudioDevice", ROBOT_IP, ROBOT_PORT)
			vol = ad.getOutputVolume()
			if msg.data == "up":
				vol = 100 if vol > 90 else vol+10
			elif msg.data == "down":
				vol = 0 if vol < 10 else vol-10
			ad.setOutputVolume(vol)
		return IsDone("Set", "ChangeVolume")
	

	# Define event to terminate subscriber spin on command
	event = threading.Event()
	event.set()

	# Initiate subscriber without listening
	# spin implemented in thread so it can be terminated on command
	def subscribe_listen(init_subscriber):
		init_subscriber()
		while event.is_set() and not rospy.is_shutdown():
			rospy.rostime.wallsleep(0.5)

	# General TTS to be used by any module
	thread_tts = threading.Thread(target=subscribe_listen, args=(
		lambda: StringSubscriber(TTS_TOPIC, tts_callback, listen=0, log=False),
		))

	# ALAudioPlayer used by speech module
	thread_ap = threading.Thread(target=subscribe_listen, args=(
		lambda: StringSubscriber(AUDIO_PLAYER_TOPIC, audio_player_callback, listen=0, log=False),
		))

	# Hands info sent by CV module
	thread_point = threading.Thread(target=subscribe_listen, args=(
		lambda: CVInfoSubscriber(POINT_TOPIC, point_callback, listen=0, log=False),
		))

	# Change volume command called by NLP
	thread_volume = threading.Thread(target=subscribe_listen, args=(
		lambda: StringSubscriber(VOLUME_TOPIC, volume_callback, listen=0, log=False),
		))
	

	# Run threads
	thread_tts.start()
	thread_ap.start()
	thread_point.start()
	thread_volume.start()

	# Exit when KeyboardInterrupt or when killed
	global kill_threads
	try:
		while True:
			if not kill_threads:
				continue
			if not TEST_DUMMY:
				try:
					ap = ALProxy("ALAudioPlayer", ROBOT_IP, ROBOT_PORT)
					ap.stopAll()
				except:
					pass
			event.clear()
			thread_tts.join()
			thread_ap.join()
			thread_point.join()
			thread_volume.join()
			break
	except KeyboardInterrupt:
		print("KeyboardInterrupt")
		event.clear()
		thread_tts.join()
		thread_ap.join()
		thread_point.join()
		thread_volume.join()

	return

# Kill Action.Listen() and Action.Localize()
kill_threads = False
def KillThreads():
	global kill_threads
	kill_threads = True

# ==============================================================
# Get status action. IsDone=False means action is still running.
# Used to know when an Action has been completed

action_status_dict = {
	"ALTextToSpeech": False,
	"ALAudioPlayer": False,
	"Point": False,
	"ChangeVolume": False
}
def IsDone(action, name):
	
	global action_status_dict

	if action == "Get":
		return action_status_dict[name]

	if action == "Reset":
		action_status_dict[name] = False

	if action == "Set":
		action_status_dict[name] = True

	return

def signal_handler(sig, frame):
	global interrupt
	interrupt = True
signal.signal(signal.SIGUSR1, signal_handler)
interrupt = False

# ==============================================================
# Localization with ArUco Markers

LOCALIZING = False
def Localize():

	if TEST_DUMMY:
		return

	from naoqi import ALProxy
	import vision_definitions
	import Image
	import numpy as np
	import cv2
	from .utils import pose_estimation

	global D, LOCALIZING

	def initialize_video_proxy():
		subscriberID = "subscriberID"
		# video = cv2.VideoCapture(0)
		video = ALProxy("ALVideoDevice", ROBOT_IP, ROBOT_PORT)
		cameraIndex = 0
		resolution = vision_definitions.kQVGA
		colorSpace = vision_definitions.kRGBColorSpace
		fps = 10
		subscriberID = video.subscribeCamera(subscriberID, cameraIndex, resolution, colorSpace, fps)
		return video, subscriberID
		

	aruco_dict_type = cv2.aruco.DICT_ARUCO_ORIGINAL
	k = np.load("data/camera_matrix.npy")
	d = np.load("data/dist_coeffs.npy")

	video, subscriberID = initialize_video_proxy()

	global kill_threads
	try:
		while not kill_threads:
			try:
				img = video.getImageRemote(subscriberID)
				if img is None:
					LOCALIZING = False
					continue
				LOCALIZING = True
				imgWidth, imgHeight = img[0], img[1]
				array = img[6]
				im = Image.frombytes("RGB", (imgWidth, imgHeight), array)
				frame = np.array(im)[:,:,::-1] # convert to BGR
				frame = frame.astype(np.uint8)
				output, Dnew = pose_estimation(frame, (imgWidth,imgHeight), aruco_dict_type, k, d)
				if Dnew is not None:
					D = Dnew
				output = cv2.resize(output, (640,480), cv2.INTER_AREA)
				cv2.imshow('Estimated Pose', output)
				key = cv2.waitKey(1) & 0xFF
				if key == ord('q'):
					break
			except RuntimeError:
				LOCALIZING = False
				video, subscriberID = initialize_video_proxy()
	except KeyboardInterrupt:
		print("KeyboardInterrupt")
		video.releaseImage(subscriberID)
		video.unsubscribe(subscriberID)
		cv2.destroyAllWindows()
