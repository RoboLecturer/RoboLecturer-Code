import PepperAPI
from PepperAPI import Info

def web_main():

	# ========= STATE: Start =========
	# Wait for signal that loop has started
	if not Info.Request("State", {"name":"Start"}):
		return

	# Change slide, then send slides text to NLP
	change_slide()
	slides = ""
	Info.Send("Slides", {"text": slides})


	# ========= STATE: AnyQuestions =========
	# Nothing to do here. Just wait for state change to "NoHandsRaised"
	state_any_questions = Info.Request("State", {"name":"AnyQuestions"})
	while state_any_questions != "NoHandsRaised":
		state_any_questions = Info.Request("State", {"name":"AnyQuestions"})
		time.sleep(.5)


	# ========= STATE: NoiseLevel =========
	# Wait for state update
	state_noise_level = Info.Request("State", {"name":"NoiseLevel"})
	while not state_noise_level:
		state_noise_level = Info.Request("State", {"name":"NoiseLevel"})
		time.sleep(.5)

	# If high noise level, pepper tells joke/shutup, then loop restarts
	if state_noise_level == "High":
		return

	# Else if low noise level, proceed to next state


	# ========= STATE: Attentiveness =========
	# Wait for state update
	state_attentiveness = Info.Request("State", {"name":"Attentiveness"})
	while not state_attentiveness:
		state_attentiveness = Info.Request("State", {"name":"Attentiveness"})
		time.sleep(.5)

	# If not attentive, get signal from Control to trigger joke or quiz
	# If trigger_quiz, trigger the quiz
	if state_attentiveness == "NotAttentive":
		signal = Info.Request("TriggerJokeOrQuiz")
		
		# If trigger_joke, send joke to Speech
		if signal == "quiz":
			# Do trigger quiz, then send take_control signal back to Control
			Info.Send("TakeControl")

		# After quiz is triggered, or if trigger_joke, loop restarts
		return

	# Else if attentive, proceed


	# ========= STATE: NoQuestionsLoop =========
	# Wait for state update from master to check if no_questions_loop has reached counter threshold
	state_no_questions_loop = Info.Request("State", {"name":"NoQuestionsLoop"})
	while not state_no_questions_loop:
		state_no_questions_loop = Info.Request("State", {"name":"NoQuestionsLoop"})
		time.sleep(.5)
	
	# If loop counter reached, Control triggers joke or quiz and loop restarts
	if state_no_questions_loop == "CounterReached":
		signal = Info.Request("TriggerJokeOrQuiz")
		if signal == "quiz":
			Info.Send("TakeControl")
		return

	# Else, restart loop
	return
		

# =================================================

if __name__ == "__main__":
	PepperAPI.init("web_node")
	while True:
		web_main()
