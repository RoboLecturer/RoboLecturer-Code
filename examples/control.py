import PepperAPI
from PepperAPI import Action, Info
import threading

# Counter and threshold for number of loops with no questions
no_questions_counter = 0
no_questions_threshold = 3

def main():

	global no_questions_counter, no_questions_threshold

	resetAllStates()

	# ========= STATE: Start =========
	Info.Send("State", {"Start":"1"})

	# Send signal to Web to increment slide
	Info.Send("ChangeSlide", {"value":"increment|0"})

	# Wait for Pepper to finish delivering slides
	Action.IsDone("Reset", "ALAudioPlayer")
	while not Action.IsDone("Get", "ALAudioPlayer"):
		pass
	
	# Then tell CV to start detecting for raised hands
	Info.Send("TriggerHandDetection")


	# ========= STATE: AnyQuestions =========
	resetState("Start")

	# Wait for state update from CV
	state = Info.Request("State", {"name":"AnyQuestions"})
	
	# If no raised hands detected, increment no_questions_counter
	if state == "NoHandsRaised":
		no_questions_counter += 1

	# Else if hands raised
	else:

		# Store hands info in list
		hands_info_list = Info.Request("RaisedHandInfo")
		
		# Start QnA loop
		for i in range(len(hands_info_list)):
			# Point, then send trigger_listen to Speech
			hand_info = hands_info_list[i]
			Action.IsDone("Reset", "Point")
			Action.Request("Point", {"info": hand_info})
			while not Action.IsDone("Get", "Point"):
				pass
			Info.Send("TriggerListen")

			# Wait for answer to be finished playing
			Action.IsDone("Reset", "ALAudioPlayer")
			while not Action.IsDone("Get", "ALAudioPlayer"):
				pass

			# When it's the last hand, update the state 
			if i == (len(hands_info_list) - 1):
				Info.Send("State", {"AnyQuestions":"NoHandsRaised", "print":False})
			else:
				Info.Send("State", {"AnyQuestions":"HandsRaised", "print":False})


	# If no hands detected, or When QnA loop ends, proceed


	# ========= STATE: NoiseLevel =========
	# Wait for state update from Speech
	state = Info.Request("State", {"name": "NoiseLevel"})

	# If high noise level, send trigger_joke/shutup to NLP,
	# then play audio from Speech
	if state == "High":
		Info.Send("TriggerJokeOrShutup")
		Action.IsDone("Reset", "ALAudioPlayer")
		while not Action.IsDone("Get", "ALAudioPlayer"):
			pass
		return

	# Else if low noise level, proceed


	# ========= STATE: Attentiveness =========
	# Wait for state update from CV
	state = Info.Request("State", {"name": "Attentiveness"})

	# If inattentive, trigger joke (NLP) or trigger quiz (Web)
	if state == "NotAttentive":
		signal = Info.Send("TriggerJokeOrQuiz")

		# If trigger_joke, play audio from Speech
		if signal == "joke":
			Action.IsDone("Reset", "ALAudioPlayer")
			while not Action.IsDone("Get", "ALAudioPlayer"):
				pass

		# Else if trigger_quiz, wait for take_control from Web
		elif signal == "quiz":
			Info.Request("TakeControl")	
		
		# Restart loop after joke is played or quiz ended
		return

	# Else, proceed


	# ========= STATE: NoQuestionsLoop =========
	# If no_questions_counter reaches threshold, update state,
	# then trigger joke (NLP) or quiz (Web)
	if no_questions_counter >= no_questions_threshold:
		no_questions_counter = 0
		Info.Send("State", {"NoQuestionsLoop": "CounterReached"})
		signal = Info.Send("TriggerJokeOrQuiz")
		if signal == "joke":
			Action.IsDone("Reset", "ALAudioPlayer")
			while not Action.IsDone("Get", "ALAudioPlayer"):
				pass
		elif signal == "quiz":
			Info.Request("TakeControl")	
		return

	else:
		Info.Send("State", {"NoQuestionsLoop": "Continue"})

	# Restart loop
	return


# =================================================

# Helper functions for resetting states

def resetState(name):
	Info.Send("State", {name: ""})

def resetAllStates():
	states = ["AnyQuestions","NoiseLevel","Attentiveness","NoQuestionsLoop"]
	for state in states:
		resetState(state)

# =================================================

if __name__ == "__main__":
	PepperAPI.init("test")
	t1 = threading.Thread(target=lambda: Action.Listen())
	t2 = threading.Thread(target=lambda: Info.Listen())
	t1.start()
	t2.start()
	while True:
		main()
