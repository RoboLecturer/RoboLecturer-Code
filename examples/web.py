import PepperAPI
from PepperAPI import Info

LOOP_COUNT = 0
def web_main():

	LOOP_COUNT += 1
	# ========= STATE: Start =========
	# Wait for signal that loop has started
	Info.Request("State", {"name":"Start"})

	if LOOP_COUNT == 1: # happens only in the very first loop
		# Send text for all slides to Web one by one
		for slide_text in list_of_slides_text:
			# TODO: Send slides text to NLP
			slides_text = "These are the new slides."
			Info.Send("Slides", {"text": slides_text})
		Info.Send("Slides", {"text": "DONE"}) # tell NLP that you've finished sending


	# ========= STATE: AnyQuestions =========
	# Nothing to do here. Just wait for state change to "NoHandsRaised"
	state = Info.Request("State", {"name":"AnyQuestions"})
	while state != "NoHandsRaised":
		state = Info.Request("State", {"name":"AnyQuestions", "print":False})


	# ========= STATE: NoiseLevel =========
	# Wait for state update
	state = Info.Request("State", {"name":"NoiseLevel"})

	# If high noise level, pepper tells joke/shutup, then loop restarts
	if state == "High":
		return

	# Else if low noise level, proceed to next state


	# ========= STATE: Attentiveness =========
	# Wait for state update
	state = Info.Request("State", {"name":"Attentiveness"})

	# If not attentive, get signal from Control to trigger joke or quiz
	if state == "NotAttentive":
		signal = Info.Request("TriggerJokeOrQuiz")
		
		# If trigger_quiz, start quiz
		if signal == "quiz":
			# TODO: trigger quiz
			# Send take_control signal back to Control
			Info.Send("TakeControl")

		# After quiz is triggered, or if trigger_joke, loop restarts
		return

	# Else if attentive, proceed


	# ========= STATE: NoQuestionsLoop =========
	# Wait for state update from master to check if no_questions_loop has reached counter threshold
	state = Info.Request("State", {"name":"NoQuestionsLoop"})
	
	# If loop counter reached, Control triggers joke or quiz and loop restarts
	if state == "CounterReached":
		signal = Info.Request("TriggerJokeOrQuiz")
		if signal == "quiz":
			# TODO: trigger quiz
			# Send take_control signal back to Control
			Info.Send("TakeControl")
		return

	# Else, restart loop
	return
		

# =================================================

if __name__ == "__main__":
	PepperAPI.init("test")
	while True:
		web_main()
