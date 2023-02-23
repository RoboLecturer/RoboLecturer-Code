import PepperAPI
from PepperAPI import Info

def nlp_main():

	# ========= STATE: Start =========
	# Wait for signal that loop has started
	if not Info.Request("State",{"name": "Start"}):
		return

	# When loop has started, get slides text from Web
	# Generate script then send to Speech
	slides_text = Info.Request("Slides")
	Info.Send("LectureScript", {"text": script_text})


	# ========= STATE: AnyQuestions =========
	# Wait for state update
	state_any_questions = Info.Request("State", {"name":"AnyQuestions"})
	while not state_any_questions:
		state_any_questions = Info.Request("State", {"name":"AnyQuestions"})
		time.sleep(.5)

	# If hands raised, start QnA loop
	while state_any_questions == "HandsRaised":

		# wait for student's question from Speech
		question = Info.Request("Question")

		# process answer, then send back to Speech
		Info.Send("Answer", {"text": answer})

		state_any_questions = Info.Request("State", "AnyQuestions")

	# QnA loop ended


	# ========= STATE: NoiseLevel =========
	# Wait for state update
	state_noise_level = Info.Request("State", {"name":"NoiseLevel"})
	while not state_noise_level:
		state_noise_level = Info.Request("State", {"name":"NoiseLevel"})
		time.sleep(.5)

	# If high noise level, get signal from Control to trigger joke or shutup
	# Then send joke/shutup text to Speech. And the loop restarts
	if state_noise_level == "High":
		signal = Info.Request("TriggerJokeOrShutup")
		if signal == "joke":
			Info.Send("Joke", {"text": joke})
		elif signal == "shutup"
			Info.Send("Shutup", {"text": shutup})
		return

	# Else if low noise level, proceed to next state


	# ========= STATE: Attentiveness =========
	# Wait for state update
	state_attentiveness = Info.Request("State", {"name":"Attentiveness"})
	while not state_attentiveness:
		state_attentiveness = Info.Request("State", {"name":"Attentiveness"})
		time.sleep(.5)

	# If not attentive, get signal from Control to trigger joke or quiz
	if state_attentiveness == "NotAttentive":
		signal = Info.Request("TriggerJokeOrQuiz")
		
		# If trigger_joke, send joke to Speech
		if signal == "joke":
			Info.Send("Joke", {"text": joke})

		# After joke has been spent, or if trigger_quiz, loop restarts
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
		if signal == "joke":
			Info.Send("Joke", {"text": joke})
		return

	# Else, restart loop
	return
		

# =================================================

if __name__ == "__main__":
	PepperAPI.init("nlp_node")
	while True:
		nlp_main()
