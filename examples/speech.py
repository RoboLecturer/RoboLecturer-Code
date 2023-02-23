import PepperAPI
from PepperAPI import Action, Info

def speech_main():

	# ========= STATE: Start =========
	# Wait for signal that loop has started
	if not Info.Request("State", {"name":"Start"}):
		return

	# When loop has started, wait for script from NLP
	# Then convert to MP3 and send to Kinematics
	script = Info.Request("LectureScript")
	Action.Request("ALAudioPlayer", {"path": "/path/to/script")


	# ========= STATE: AnyQuestions =========
	# Wait for state on hands raised or not
	state_any_questions = Info.Request("State", {"name":"AnyQuestions"})
	while not state_any_questions:
		state_any_questions = Info.Request("State", {"name":"AnyQuestions"})
		time.sleep(.5)

	# If hands raised, start QnA loop
	while state_any_questions == "HandsRaised":

		# wait for signal from kinematics to start listening to mic
		if not Info.Request("TriggerListen"):
			pass

		# Listen to mic and process question STT
		# Then send STT to NLP
		Info.Send("Question", {"text": question})

		# Now wait for answer from NLP
		# Convert to audio and send to Kinematics
		answer = Info.Request("Answer")
		Action.Request("ALAudioPlayer", {"path": "/path/to/answer"}

		state_any_questions = Info.Request("State", "AnyQuestions")

	# When QnA loop ends, proceed


	# ========= STATE: NoiseLevel =========
	# Start detecting noise

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
		Action.Request("ALAudioPlayer", {"path": "/path/to/text"})

		# then loop restarts
		return

	# Else, update state NoiseLevel accordingly
	Info.Send("State", {"NoiseLevel": "Low"})


	# ========= STATE: Attentiveness =========
	# If low noise level, CV starts attentiveness detection 
	# Wait for update on state change
	state_attentiveness = Info.Request("State", {"name": "Attentiveness"})
	while not state_attentiveness:
		state_attentiveness = Info.Request("State", {"name": "Attentiveness"})
		time.sleep(.5)

	# If inattentive, Control trigger joke or quiz
	if state_attentiveness == "NotAttentive":
		signal = Info.Request("TriggerJokeOrQuiz")

		# If trigger_joke, receive joke from NLP, convert to audio and send to play
		if signal == "joke":
			joke = Info.Request("Joke")
			Action.Request("ALAudioPlayer", {"path": "/path/to/joke"})
		
		# Restart loop after joke is played or if trigger_quiz
		return

	# Else, proceed


	# ========= STATE: NoQuestionsLoop =========
	# Wait for state update from master to check if no_questions_loop has reached counter threshold
	state_no_questions_loop = Info.Request("State", {"name":"NoQuestionsLoop"})
	while not state_no_questions_loop:
		state_no_questions_loop = Info.Request("State", {"name":"NoQuestionsLoop"})
		time.sleep(.5)
	
	# If loop counter reached, Control triggers joke or quiz and loop restarts
	if state_no_questions_loop == "CounterReached":
		signal = Info.Request("TriggerJokeOrQuiz")
		if signal == "joke":
			joke = Info.Request("Joke")
			Action.Request("ALAudioPlayer", {"path": "/path/to/joke"})
		return

	# Else, restart loop
	return
		

# =================================================

if __name__ == "__main__":
	PepperAPI.init("speech_node")
	while True:
		speech_main()
