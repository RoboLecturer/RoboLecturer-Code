import PepperAPI
from PepperAPI import Action, Info
import random
from text2speech import *
import speech2text as sp
import Class_noise_detection as nd

def speech_main():

	# ========= STATE: Start =========
	# Wait for signal that loop has started
	Info.Request("State", {"name":"Start"})

	# When loop has started, wait for script from NLP
	# Then convert to MP3 and send to Kinematics
	script = Info.Request("LectureScript")
	runT2S(script)
	# TODO: convert lecture script to audio and save somewhere in your machine
	path_to_audio = "output.wav"
	Action.Request("ALAudioPlayer", {"path": path_to_audio})


	# ========= STATE: AnyQuestions =========
	# Wait for state on hands raised or not
	state = Info.Request("State", {"name":"AnyQuestions"})

	# If hands raised, start QnA loop
	while state == "HandsRaised":

		# wait for signal from kinematics to start listening to mic
		Info.Request("TriggerListen")

		# TODO: Listen to mic and process question STT,
		question = sp.runSTT(0)
		# then send STT to NLP
		Info.Send("Question", {"text": question})

		# Wait for answer from NLP, 
		answer = Info.Request("Answer")
		runT2S(answer)
		# TODO: convert answer to audio and save somewhere in your machine
		Action.Request("ALAudioPlayer", {"path": path_to_audio})

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
		
		runT2S(text)
	
		# TODO: convert joke/shutup text into audio and save
		Action.Request("ALAudioPlayer", {"path": path_to_audio})

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
			runT2S(joke)
			# TODO: convert joke/shutup text into audio and save
			Action.Request("ALAudioPlayer", {"path": path_to_audio})
		
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
			runT2S(joke)
			# TODO: convert joke/shutup text into audio and save
			Action.Request("ALAudioPlayer", {"path": path_to_audio})
		return

	# Else, restart loop
	return
		

# =================================================

if __name__ == "__main__":
	PepperAPI.init("test")
	while True:
		speech_main()