import PepperAPI
from PepperAPI import Action, Info
import threading, time

# Counter for number of loops with no questions
no_questions_counter = 0

def main():

	global no_questions_counter

	Info.resetAllStates()

	# ========= STATE: Start =========
	Info.Send("State", {"Start":"1"})
	print("\n========= STATE: Start =========")

	# Wait for Pepper to finish delivering slides
	Action.IsDone("Reset", "ALAudioPlayer")
	while not Action.IsDone("Get", "ALAudioPlayer"):
		pass
	
	# Then tell CV to start detecting for raised hands
	Info.Send("TriggerHandDetection")


	# ========= STATE: AnyQuestions =========
	Info.resetState("Start")

	# Wait for state update from CV
	state_any_questions = Info.Request("State", {"name":"AnyQuestions"})
	while not state_any_questions:
		state_any_questions = Info.Request("State", {"name":"AnyQuestions"})
	print("\n========= STATE: AnyQuestions =========")
	
	# If hands raised
	if state_any_questions == "HandsRaised":

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

			# When it's the last hand, update the state 
			if i == (len(hands_info_list) - 1):
				Info.Send("State", {"AnyQuestions": "NoHandsRaised"})

			# Wait for answer to be finished playing
			Action.IsDone("Reset", "ALAudioPlayer")
			while not Action.IsDone("Get", "ALAudioPlayer"):
				pass


	# If no hands detected, or When QnA loop ends, proceed


	# ========= STATE: NoiseLevel =========
	# Wait for state update from Speech
	state_noise_level = Info.Request("State", {"name": "NoiseLevel"})
	while not state_noise_level:
		state_noise_level = Info.Request("State", {"name": "NoiseLevel"})
	print("\n========= STATE: NoiseLevel =========")

	# If high noise level, send trigger_joke/shutup to NLP,
	# then play audio from Speech
	if state_noise_level == "High":
		Info.Send("TriggerJokeOrShutup")
		Action.IsDone("Reset", "ALAudioPlayer")
		while not Action.IsDone("Get", "ALAudioPlayer"):
			pass
		return

	# Else if low noise level, proceed


	# ========= STATE: Attentiveness =========
	# Wait for state update from CV
	state_attentiveness = Info.Request("State", {"name": "Attentiveness"})
	while not state_attentiveness:
		state_attentiveness = Info.Request("State", {"name": "Attentiveness"})
	print("\n========= STATE: Attentiveness =========")

	# If inattentive, trigger joke (NLP) or trigger quiz (Web)
	if state_attentiveness == "NotAttentive":
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
	print("\n========= STATE: NoQuestionsLoop =========")
	# If no_questions_counter reaches threshold, update state,
	# then trigger joke (NLP) or quiz (Web)
	if no_questions_counter == threshold:
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
		no_questions_counter += 1

	# Restart loop
	return


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
	t2 = threading.Thread(target=lambda: Info.Broadcast())
	t1.start()
	t2.start()
	while True:
		main()
