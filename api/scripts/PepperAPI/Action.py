from .Publisher import *
import paramiko


def Request(api_name, api_params):
	
	# Global parameters
	ROBOT_IP = "192.168.0.102"
	ROBOT_PORT = 9559

	# API callbacks
	"""
	Send message to be said by Pepper
	@param api_params: {
		"value": message to be sent
	}
	"""
	
	# Audio APIs
	if api_name == "ALTextToSpeech":
		msg = api_params["value"]
		tts = ALProxy("ALTextToSpeech", ROBOT_IP, ROBOT_PORT)
		tts.say(msg)
		
	if api_name == "ALAudioPlayer":
		soundfile_path = api_params["path"]
		filename = soundfile_path.split("/")[-1]
		pepper_path = "/home/user/" + filename
		transport = paramiko.Transport((ROBOT_IP, 22))
		transport.connect(username="nao", password="BioARTLab123")
		sftp = paramiko.SFTPClient.from_transport(transport)
		sftp.put(soundfile_path, pepper_path)
		sftp.close()
		transport.close()
		play_audio_publisher.publish(pepper_path)

	return
