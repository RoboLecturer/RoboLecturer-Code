import PepperAPI
from PepperAPI import Action, Info
import random
from mutagen.mp3 import MP3
from mutagen.flac import FLAC

LOOP_COUNT = 0
def speech_main():

	global LOOP_COUNT

	LOOP_COUNT += 1

	# ========= STATE: Start =========
	# Wait for signal that loop has started
	Info.Request("State", {"name":"Start"})

	# When loop has started, wait for script from NLP
	# Then convert to MP3 and send to Kinematics
	if LOOP_COUNT == 1:
		script = Info.Request("LectureScript")

	# TODO: convert lecture script to audio and save somewhere in your machine
	path_to_audio = "/home/user/Downloads/statquest.mp3"
	audio = MP3(path_to_audio)
	audio_file_length = audio.info.length

	# Action.Request("ALAudioPlayer", {"file": "output0.wav"})
	Action.Request("ALAudioPlayer", {
		"path": path_to_audio,
		"length": audio_file_length
		})


	# ========= STATE: AnyQuestions =========
	# Wait for state on hands raised or not
	state = Info.Request("State", {"name":"AnyQuestions"})

	# If hands raised, start QnA loop
	while state == "HandsRaised":

		# wait for signal from kinematics to start listening to mic
		Info.Request("TriggerListen")

		# TODO: Listen to mic and process question STT,
		question = "Why is the sky blue?"
		# then send STT to NLP
		Info.Send("Question", {"text": question})

		# Wait for answer from NLP, 
		answer = Info.Request("Answer")
		# TODO: convert answer to audio and save somewhere in your machine
		path_to_audio = "/home/user/sample.flac"
		audio = FLAC(path_to_audio)
		audio_file_length = audio.info.length
		Action.Request("ALAudioPlayer", {
			"path": path_to_audio,
			"length": audio_file_length
			})

		state = Info.Request("State", {"name":"AnyQuestions", "print":False})

	# When QnA loop ends, proceed


	# ========= STATE: NoiseLevel =========
	# TODO: Start detecting noise and classify into high or low noise level
	HIGH_NOISE_LEVEL = random.choice([True, False])

	# If high noise level, update state.
	# Control tells NLP to trigger joke/shutup, you receive text,
	# convert to audio and send to Kinematics to play
	if HIGH_NOISE_LEVEL:
		Info.Send("State", {"NoiseLevel": "High"})
		signal = Info.Request("TriggerJokeOrShutup")
		if signal == "joke":
			text = Info.Request("Joke")
		else:
			text = Info.Request("Shutup")
	
		# TODO: convert joke/shutup text into audio and save
		path_to_audio = "/home/user/sample.flac"
		audio = FLAC(path_to_audio)
		audio_file_length = audio.info.length
		Action.Request("ALAudioPlayer", {
			"path": path_to_audio,
			"length": audio_file_length
			})

		# then loop restarts
		return

	# Else, update state NoiseLevel accordingly
	Info.Send("State", {"NoiseLevel": "Low"})


	# ========= STATE: Attentiveness =========
	# If low noise level, CV starts attentiveness detection 
	# Wait for update on state change
	state = Info.Request("State", {"name": "Attentiveness"})
	
	# If inattentive, Control trigger joke or quiz
	if state == "NotAttentive":
		signal = Info.Request("TriggerJokeOrQuiz")

		# If trigger_joke, receive joke from NLP, convert to audio and send to play
		if signal == "joke":
			joke = Info.Request("Joke")
			# TODO: convert joke/shutup text into audio and save
			path_to_audio = "/home/user/sample.flac"
			audio = FLAC(path_to_audio)
			audio_file_length = audio.info.length
			Action.Request("ALAudioPlayer", {
				"path": path_to_audio,
				"length": audio_file_length
				})
		
		# Restart loop after joke is played or if trigger_quiz
		return

	# Else, proceed


	# ========= STATE: NoQuestionsLoop =========
	# Wait for state update from master to check if no_questions_loop has reached counter threshold
	state = Info.Request("State", {"name":"NoQuestionsLoop"})
	
	# If loop counter reached, Control triggers joke or quiz and loop restarts
	if state == "CounterReached":
		signal = Info.Request("TriggerJokeOrQuiz")
		if signal == "joke":
			joke = Info.Request("Joke")
			# TODO: convert joke/shutup text into audio and save
			path_to_audio = "/home/user/sample.flac"
			audio = FLAC(path_to_audio)
			audio_file_length = audio.info.length
			Action.Request("ALAudioPlayer", {
				"path": path_to_audio,
				"length": audio_file_length
				})
		return

	# Else, restart loop
	return
		

# =================================================

if __name__ == "__main__":
	PepperAPI.init("master")
	while True:
		speech_main()
