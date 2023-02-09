from .Publisher import *
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

	def audio_player_callback(data):
		filename = data.data
		rospy.loginfo("Play audio: %s" % filename)
		ap = ALProxy("ALAudioPlayer", ROBOT_IP, ROBOT_PORT)
		audio_file = PEPPER_AUDIO_PATH + filename
		ap.post.playFile(audio_file)

	def point_callback(data):
		x,y = data.x, data.y
		rospy.loginfo("Point at x=%.3f y=%.3f" % (x,y))	

	# General TTS to be used by any module
	StringSubscriber("tts_topic", tts_callback, listen_once=False)

	# ALAudioPlayer used by speech module
	StringSubscriber("audio_player_topic", audio_player_callback, listen_once=False)
	
	# Coords sent by CV module
	CoordsSubscriber("point_topic", point_callback, listen_once=False)
