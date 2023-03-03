from __future__ import print_function
import sys
import time
import threading
import rospy
import paramiko
from .Publisher import *
from .Subscriber import *
from PepperAPI import *

# =========================================================
# Request for actuator action by Pepper robot

def Request(api_name, api_params={}):

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
		}
		"""
		def printProgress(transferred, left):
			print("Transferred: {0}\tOut of: {1}".format(transferred, left), end="\r")

		if "file" not in api_params:
			# Define paths
			soundfile_path = api_params["path"]
			filename = soundfile_path.split("/")[-1]
			pepper_path = PEPPER_AUDIO_PATH + filename

			# Setup SFTP link
			transport = paramiko.Transport((ROBOT_IP, 22))
			transport.connect(username=PEPPER_USER, password=PEPPER_PASSWORD)
			transport.default_max_packet_size = 1000000000
			transport.default_window_size = 1000000000
			sftp = paramiko.SFTPClient.from_transport(transport)

			# Send file
			sftp.put(soundfile_path, pepper_path, callback=printProgress)
			sftp.close()
			transport.close()

		else:
			filename = api_params["file"]

		# Publish msg to kinematics module to play audio
		audio_player_publisher.publish(filename)
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

def Listen():

	# Import NAOqi modules
	from naoqi import ALProxy, ALBroker

	# Callback for ALTextToSpeech
	def tts_callback(msg):
		rospy.loginfo("Pepper ALTextToSpeech: Say %s" % msg.data)
		tts.say(msg.data)
		return IsDone("Set", "ALTextToSpeech")	

	# Callback for ALAudioPlayer
	def audio_player_callback(msg):
		filename = msg.data
		rospy.loginfo("Pepper ALAudioPlayer: Play audio %s" % filename)
		audio_file = PEPPER_AUDIO_PATH + filename
		ap.playFile(audio_file)
		# time.sleep(5)
		return IsDone("Set", "ALAudioPlayer")	

	# Callback for pointing at raised hand
	def point_callback(msg):
		# parse msg
		x,y,w,h = msg.x, msg.y, msg.w, msg.h
		frame_width = msg.frame_width
		frame_height = msg.frame_height

		# constants
		Z_UP = 1.0
		Z_DOWN = 0.7
		Y_LARM_OUT = 5.0
		Y_LARM_IN = 0
		Y_RARM_OUT = -Y_LARM_OUT
		Y_RARM_IN = -Y_LARM_IN

		# parameters
		center_x = x + w//2
		center_y = y + h//2
		mid_width = frame_width // 2
		rospy.loginfo("Raised hand at (%.0f, %.0f) of (%d, %d)" %
			(center_x, center_y, frame_width, frame_height))

		# point parameters
		max_speed = 0.5
		frame = 0 # Torso=0, World=1, Robot=2

		# point in/out
		point_x = 1.0 # fixed

		# point left/right
		if center_x < mid_width:
			effector = "LArm"
			point_y = Y_LARM_OUT - (Y_LARM_OUT - Y_LARM_IN) * center_x / mid_width
		else:
			effector = "RArm"
			point_y = Y_RARM_OUT + (Y_RARM_IN - Y_RARM_OUT) * (center_x - mid_width) / mid_width

		# point up/down
		point_z = Z_UP - Z_DOWN * center_y / frame_height

		# actuate
		rospy.loginfo("Pepper ALTracker: Point %s at x=%.2f y=%.2f, z=%.2f" % 
			(effector, point_x, point_y, point_z))

		# posture.applyPosture("StandInit", 0.5)
		tracker.lookAt([point_x,point_y,point_z], frame, max_speed, False)
		tracker.pointAt(effector, [point_x,point_y,point_z], frame, max_speed)
		ap.playFile(PEPPER_AUDIO_PATH + "what_is_your_qn.wav")

		return IsDone("Set", "Point")

	# Increase/decrease master volume
	def volume_callback(msg):
		rospy.loginfo("Pepper ALAudioDevice: Volume " + msg.data)
		vol = ad.getOutputVolume()
		if msg.data == "up":
			vol = 100 if vol > 90 else vol+10
		elif msg.data == "down":
			vol = 0 if vol < 10 else vol-10
		ad.setOutputVolume(vol)
		return IsDone("Set", "ChangeVolume")


	# Initialise proxies
	try:
		broker = ALBroker("broker", "0.0.0.0", 54000, ROBOT_IP, ROBOT_PORT)
		tts = ALProxy("ALTextToSpeech")
		ap = ALProxy("ALAudioPlayer")
		tracker = ALProxy("ALTracker")
		posture = ALProxy("ALRobotPosture")
		ad = ALProxy("ALAudioDevice")
	except Exception as e:
		print(e)
		sys.exit()


	# Define event to terminate thread on command
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
			try:
				ap.stopAll()
			except:
				pass
			event.clear()
			thread_tts.join()
			thread_ap.join()
			thread_point.join()
			thread_volume.join()
			broker.shutdown()
			break
	except KeyboardInterrupt:
		print("KeyboardInterrupt")
		event.clear()
		thread_tts.join()
		thread_ap.join()
		thread_point.join()
		thread_volume.join()

	return

# Kill Action.Listen()
kill_threads = False
def KillThreads():
	global kill_threads
	kill_threads = True

# ==============================================================
# Get status action. IsDone=False means action is still running.

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
