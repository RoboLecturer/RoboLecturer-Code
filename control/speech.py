import PepperAPI
from PepperAPI import Action, Info
import random
import os

if not PepperAPI.TEST_DUMMY:
	from mutagen.mp3 import MP3
	from mutagen.flac import FLAC

# Set to True if you want to input your own questions
INPUT_QUESTION = True

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

	path_to_audio = "/home/user/Downloads/sample.flac"
	audio_file_length = get_audio_length(path_to_audio)
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

		if INPUT_QUESTION:
			question = input("Question: ")
		else:
			question = "Why is the sky blue?"
		# then send STT to NLP
		Info.Send("Question", {"text": question})

		student_done = Info.Request("StudentDone")
		if not student_done:

			# Wait for answer from NLP, 
			answer = Info.Request("Answer")
			path_to_audio = "/home/user/Downloads/sample.flac"
			audio_file_length = get_audio_length(path_to_audio)
			Action.Request("ALAudioPlayer", {
				"path": path_to_audio,
				"length": audio_file_length
				})

		state = Info.Request("State", {"name":"AnyQuestions", "print":False})

	# When QnA loop ends, proceed


	# ========= STATE: NoiseLevel =========
	HIGH_NOISE_LEVEL = random.choice([True, False])
	HIGH_NOISE_LEVEL = False

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
	
		path_to_audio = "/home/user/Downloads/sample.flac"
		audio_file_length = get_audio_length(path_to_audio)
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
			path_to_audio = "/home/user/Downloads/sample.flac"
			audio_file_length = get_audio_length(path_to_audio)
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
			path_to_audio = "/home/user/Downloads/sample.flac"
			audio_file_length = get_audio_length(path_to_audio)
			Action.Request("ALAudioPlayer", {
				"path": path_to_audio,
				"length": audio_file_length
				})
			return

	# Else, restart loop
	return
		

def get_audio_length(path_to_audio):
	audio_file_length = 0
	if not PepperAPI.TEST_DUMMY and os.path.exists(path_to_audio):
		filetype = path_to_audio.split(".")[-1]
		if filetype.lower() == "mp3":
			audio = MP3(path_to_audio)
		elif filetype.lower() == "flac":
			audio = FLAC(path_to_audio)
		audio_file_length = audio.info.length
	return audio_file_length



# =================================================

if __name__ == "__main__":
	PepperAPI.init("master")
	while True:
		speech_main()
