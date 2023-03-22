import PepperAPI
from PepperAPI import Action, Info
import threading, time

# Counter and threshold for number of loops with no questions
no_questions_counter = 0
no_questions_threshold = 3

loop_count = 0
def main():

	global loop_count
	loop_count += 1

	global no_questions_counter, no_questions_threshold

	resetAllStates()

	# ========= STATE: Start =========
	Info.Send("State", {"Start":"1"})

	if loop_count == 1:
		time.sleep(10) # wait for web to reload before sending change_slide

	# Send signal to Web to increment slide
	Info.Send("ChangeSlide", {"cmd":"increment|0"})

	# Wait for Pepper to finish delivering slides
	Action.IsDone("Reset", "ALAudioPlayer")
	while not Action.IsDone("Get", "ALAudioPlayer"):
		pass
	time.sleep(2)
	Action.Request("ALAudioPlayer", {"file": "do_u_have_qns.wav"})
	
	# Then tell CV to start detecting for raised hands
	Info.Send("TriggerHandDetection")


	# ========= STATE: AnyQuestions =========
	resetState("Start")
	# raw_input()

	# Wait for state update from CV
	print("listening...")
	state = Info.Request("State", {"name":"AnyQuestions"})
	
	# If no raised hands detected, increment no_questions_counter
	if state == "NoHandsRaised":
		no_questions_counter += 1

	# Else if hands raised
	else:

		# Store hands info in list
		hands_info_list = Info.Request("RaisedHandInfo")
		pointed = False
		
		# Start QnA loop
		while True:
			# Point, then send trigger_listen to Speech
			if not pointed:
				pointed = True
				hand_info = hands_info_list[0]
				Action.IsDone("Reset", "Point")
				Action.Request("Point", {"info": hand_info})
				while not Action.IsDone("Get", "Point"):
					pass
			Info.Send("TriggerListen")

			student_done = Info.Request("StudentDone")
			if not student_done:

				# Wait for answer to be finished playing
				Action.IsDone("Reset", "ALAudioPlayer")
				while not Action.IsDone("Get", "ALAudioPlayer"):
					pass

			else:
				hands_info_list.pop(0)
				pointed = False
				print("Moving onto next student. %d students left." % len(hands_info_list))

			if not len(hands_info_list):
				Info.Send("State", {"AnyQuestions":"NoHandsRaised", "print":False})
				time.sleep(1)
				break
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
			Info.Send("TriggerQuiz")
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
			Info.Send("TriggerQuiz")
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
	PepperAPI.init("master")
	t1 = threading.Thread(target=Action.Listen)
	t2 = threading.Thread(target=Info.Listen)
	t3 = threading.Thread(target=Info.Broadcast)
	t1.start()
	t2.start()
	t3.start()

	try:
		while True:
			main()
	except KeyboardInterrupt:
		print("KeyboardInterrupt")
		Action.KillThreads()
		Info.KillThreads()
		t1.join()
		t2.join()
		t3.join()
