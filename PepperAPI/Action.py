import time
import threading
import rospy
from .Publisher import *
from .Subscriber import *
from PepperAPI import *
import paramiko

def Request(api_name, api_params):

	# API callbacks
	if api_name == "ALTextToSpeech":
		"""Send message to be said by Pepper
		@param api_params : dict{
			"value": message to be sent
		}
		"""
		msg = api_params["value"]
		tts_publisher.publish(msg)
		return True
	
	if api_name == "ALAudioPlayer":
		"""Request for audio file to be played through Pepper's speakers
		@param api_params : dict{
			"path": path of audio file in your machine
		}
		"""
		# Define paths
		soundfile_path = api_params["path"]
		filename = soundfile_path.split("/")[-1]
		pepper_path = PEPPER_AUDIO_PATH + filename

		# # Setup SFTP link
		# transport = paramiko.Transport((ROBOT_IP, 22))
		# transport.connect(username=PEPPER_USER, password=PEPPER_PASSWORD)
		# sftp = paramiko.SFTPClient.from_transport(transport)
        #
		# # Send file
		# sftp.put(soundfile_path, pepper_path)
		# sftp.close()
		# transport.close()

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

	if api_name == "VolumeUp":
		"""
		Set volume up
		"""
		volume_up_publisher.publish("1")
		return True

	if api_name == "VolumeDown":
		"""
		Set volume down
		"""
		volume_down_publisher.publish("1")
		return True

# ==============================================================
# Only to be used by Kinematics module

def Listen():

	# Import NAOqi modules
	# from naoqi import ALProxy

	# Callback for ALTextToSpeech
	def tts_callback(msg):
		rospy.loginfo("Pepper say: %s" % msg.data)
		tts = ALProxy("ALTextToSpeech", ROBOT_IP, ROBOT_PORT)
		tts.say(msg.data)
		return IsDone("Set", "ALTextToSpeech")	

	# Callback for ALAudioPlayer
	def audio_player_callback(msg):
		filename = msg.data
		rospy.loginfo("Pepper play audio: %s" % filename)
		# ap = ALProxy("ALAudioPlayer", ROBOT_IP, ROBOT_PORT)
		# audio_file = PEPPER_AUDIO_PATH + filename
		# ap.post.playFile(audio_file)
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
		rospy.loginfo("Pepper point %s at x=%.2f y=%.2f, z=%.2f" % 
			(effector, point_x, point_y, point_z))

		# tracker = ALProxy("ALTracker", ROBOT_IP, ROBOT_PORT)
		# posture = ALProxy("ALRobotPosture", ROBOT_IP, ROBOT_PORT)
		# # posture.applyPosture("StandInit", 0.5)
		# tracker.lookAt([point_x,point_y,point_z], frame, max_speed, False)
		# tracker.pointAt(effector, [point_x,point_y,point_z], frame, max_speed)

		return IsDone("Set", "Point")

	# Increase master volume
	def volume_up_callback(msg):
		ap = ALProxy("ALAudioPlayer", ROBOT_IP, ROBOT_PORT)
		vol = ap.getMasterVolume()
		ap.setMasterVolume(vol + 0.1)
		return IsDone("Set", "VolumeUp")

	# Decrease master volume
	def volume_down_callback(msg):
		ap = ALProxy("ALAudioPlayer", ROBOT_IP, ROBOT_PORT)
		vol = ap.getMasterVolume()
		ap.setMasterVolume(vol + 0.1)
		return IsDone("Set", "VolumeDown")

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
		lambda: StringSubscriber(TTS_TOPIC, tts_callback, listen=False),
		))

	# ALAudioPlayer used by speech module
	thread_ap = threading.Thread(target=subscribe_listen, args=(
		lambda: StringSubscriber(AUDIO_PLAYER_TOPIC, audio_player_callback, listen=False),
		))

	# Hands info sent by CV module
	thread_hand = threading.Thread(target=subscribe_listen, args=(
		lambda: CVInfoSubscriber(POINT_TOPIC, point_callback, listen=False),
		))

	# Volume up thread
	thread_vol_up = threading.Thread(target=subscriber_listen, args=(
		lambda: StringSubscriber(VOLUME_UP_TOPIC, volume_up_callback, listen=False),
		))

	# Volume down thread
	thread_vol_down = threading.Thread(target=subscriber_listen, args=(
		lambda: StringSubscriber(VOLUME_DOWN_TOPIC, volume_down_callback, listen=False),
		))

	# Run threads
	thread_tts.start()
	thread_ap.start()
	thread_hand.start()
	thread_vol_up.start()
	thread_vol_down.start()

	# Exit when KeyboardInterrupt
	try:
		while True:
			time.sleep(1)
	except KeyboardInterrupt:
		event.clear()
		thread_tts.join()
		thread_ap.join()
		thread_hand.join()

	return


action_status_dict = {
	"ALTextToSpeech": False,
	"ALAudioPlayer": False,
	"Point": False,
	"VolumeUp": False,
	"VolumeDown": False
}
def IsDone(action, name):
	
	global action_status_dict

	if action == "Get":
		return action_status_dict[name]

	if action == "Reset":
		action_status_dict[name] = False

	if action == "Set":
		action_status_dict[name] = True

	return True
