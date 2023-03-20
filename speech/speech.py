import sys
sys.path.append("..") # Adds higher directory to python modules path.

from mutagen.mp3 import MP3
from mutagen.wavpack import WavPack

import PepperAPI
from PepperAPI import Action, Info
import pkg.text2speech as t2s
import pkg.speech2text as s2t
import pkg.noise as nd


loop_c = 0
# TODO: use os package for OSes
OUTPUT_DIR = "output/"


def speech_main():

	global loop_c

	# Use gTTS (.mp3) or DNN (.wav)
	ONLINE=True

	# ========= STATE: Start =========
	# Wait for signal that loop has started
	Info.Request("State", {"name":"Start"})

	# When loop has started, wait for script from NLP
	# Then convert to MP3 and send to Kinematics
	if loop_c == 0:
		total_script = Info.Request("LectureScript")

		# Generate script audio for all slides
		for i, slide_script in enumerate(total_script):
			t2s.runT2S(slide_script, online=ONLINE, OUTPUT_PATH=OUTPUT_DIR+f"script_{i}")

		# Play first slide script audio
		path_to_audio = OUTPUT_DIR+"script_0"
		if ONLINE:
			path_to_audio = path_to_audio+'.mp3'
			audio = MP3(path_to_audio)
		else:
			path_to_audio = path_to_audio+'.wav'
			audio = WavPack(path_to_audio)
		print(audio.info.length)
		Action.Request("ALAudioPlayer", {"path": path_to_audio, "length": audio.info.length})
	else:
		path_to_audio = OUTPUT_DIR+f"script_{loop_c}"
		if ONLINE:
			path_to_audio = path_to_audio+'.mp3'
			audio = MP3(path_to_audio)
		else:
			path_to_audio = path_to_audio+'.wav'
			audio = WavPack(path_to_audio)
		Action.Request("ALAudioPlayer", {"path": path_to_audio, "length": audio.info.length})

	loop_c += 1
	


	# ========= STATE: AnyQuestions =========
	# Wait for state on hands raised or not
	state = Info.Request("State", {"name":"AnyQuestions"})

	# If hands raised, start QnA loop
	while state == "HandsRaised":

		# wait for signal from kinematics to start listening to mic
		Info.Request("TriggerListen")

		# TODO: Listen to mic and process question STT,
		question = s2t.runS2T(0)
		# then send STT to NLP
		Info.Send("Question", {"text": question})

		# Wait for answer from NLP, 
		path_to_answer = OUTPUT_DIR+"answer"
		answer = Info.Request("Answer")

		if ONLINE:
			path_to_answer = t2s.runT2S(answer, online=ONLINE, OUTPUT_PATH=path_to_answer)
			audio = MP3(path_to_answer)
			
			Action.Request("ALAudioPlayer", {"path": path_to_answer, "length": audio.info.length})
		else:
			sentences = [x for x in answer.split('.') if x]
			for i in range(len(sentences)):
				path_to_answer = OUTPUT_DIR+f"answer_{i}"
				path_to_answer = path_to_answer+'.wav'
				t2s.runT2S(sentences[i], online=ONLINE, OUTPUT_PATH=path_to_answer)
				audio = WavPack(path_to_answer)

				Action.Request("ALAudioPlayer", {"path": path_to_answer, "length": audio.info.length})

		state = Info.Request("State", {"name":"AnyQuestions", "print":False})

	# When QnA loop ends, proceed


	# ========= STATE: NoiseLevel =========
	# TODO: Start detecting noise and classify into high or low noise level
	HIGH_NOISE_LEVEL = nd.Shush(60, 3)
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
		
		path_to_JS = OUTPUT_DIR+"JS"
		t2s.runT2S(text, path_to_JS)
	
		# TODO: convert joke/shutup text into audio and save
		if ONLINE:
			path_to_JS = path_to_JS + '.mp3'
			audio = MP3(path_to_JS)
		else:
			path_to_JS = path_to_JS + '.wav'
			audio = WavPack(path_to_JS)
		Action.Request("ALAudioPlayer", {"path": path_to_JS, "length": audio.info.length})

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
			path_to_JS = OUTPUT_DIR+"JS"
			t2s.runT2S(joke, path_to_JS)

			# TODO: convert joke/shutup text into audio and save
			if ONLINE:
				path_to_JS = path_to_JS + '.mp3'
				audio = MP3(path_to_JS)
			else:
				path_to_JS = path_to_JS + '.wav'
				audio = WavPack(path_to_JS)
			Action.Request("ALAudioPlayer", {"path": path_to_JS, "length": audio.info.length})
		
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
			path_to_JS = OUTPUT_DIR+"JS"
			t2s.runT2S(joke, path_to_JS)

			# TODO: convert joke/shutup text into audio and save
			if ONLINE:
				path_to_JS = path_to_JS + '.mp3'
				audio = MP3(path_to_JS)
			else:
				path_to_JS = path_to_JS + '.wav'
				audio = WavPack(path_to_JS)
			Action.Request("ALAudioPlayer", {"path": path_to_JS, "length": audio.info.length})
		return

	# Else, restart loop
	return
		

# =================================================

if __name__ == "__main__":
	PepperAPI.init("test")

	while True:
		speech_main()