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

		# Setup SFTP link
		transport = paramiko.Transport((ROBOT_IP, 22))
		transport.connect(username=PEPPER_USER, password=PEPPER_PASSWORD)
		sftp = paramiko.SFTPClient.from_transport(transport)

		# Send file
		sftp.put(soundfile_path, pepper_path)
		sftp.close()
		transport.close()

		# Publish msg to kinematics module to play audio
		audio_player_publisher.publish(filename)

	return

# ==============================================================
# Only to be used by Kinematics module

def Listen():

	# Import NAOqi modules
	from naoqi import ALProxy

	def tts_callback(msg):
		tts = ALProxy("ALTextToSpeech", ROBOT_IP, ROBOT_PORT)
		tts.say(msg)
		return True

	def audio_player_callback(msg):
		filename = msg.data
		rospy.loginfo("Play audio: %s" % filename)
		ap = ALProxy("ALAudioPlayer", ROBOT_IP, ROBOT_PORT)
		audio_file = PEPPER_AUDIO_PATH + filename
		ap.post.playFile(audio_file)
		return True

	def point_callback(msg):
		# parse msg
		x,y,w,h = msg.x, msg.y, msg.w, msg.h
		frame_width = msg.frame_width
		frame_height = msg.frame_height

		# constants
		Z_UP = 1.0
		Z_DOWN = 0.5
		Y_LARM_OUT = 1.0
		Y_LARM_IN = -0.2
		Y_RARM_OUT = -Y_LARM_OUT
		Y_RARM_IN = -Y_LARM_IN

		# parameters
		center_x = x + w//2
		center_y = y + h//2
		mid_width = frame_width // 2
		rospy.loginfo("Raised hand at (%.0f, %.0f) of (%d, %d)" %
			(center_x, center_y, frame_width, frame_height))

		# point parameters
		max_speed = 0.7
		frame = 1 # Torso=0, World=1, Robot=2

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
		rospy.loginfo("Point %s at x=%.2f y=%.2f, z=%.2f" % 
			(effector, point_x, point_y, point_z))
		return

		tracker = ALProxy("ALTracker", ROBOT_IP, ROBOT_PORT)
		posture = ALProxy("ALRobotPosture", ROBOT_IP, ROBOT_PORT)

		posture.applyPosture("StandInit", 0.5)
		tracker.lookAt(effector, [x,y,z], frame, max_speed)
		tracker.pointAt(effector, [x,y,z], frame, max_speed)
		time.sleep(10)
		posture.applyPosture("StandInit", 0.5)

		return True

	# General TTS to be used by any module
	# StringSubscriber("tts", tts_callback, listen_once=False)

	# ALAudioPlayer used by speech module
	# StringSubscriber("audio_player", audio_player_callback, listen_once=False)
	
	# Hands info sent by CV module
	CVInfoSubscriber("raised_hand", point_callback)
