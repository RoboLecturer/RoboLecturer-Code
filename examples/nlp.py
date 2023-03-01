import PepperAPI
from PepperAPI import Action, Info
import random 

LOOP_COUNT = 0
list_of_scripts = []

def nlp_main():
	
	global LOOP_COUNT, list_of_scripts
	LOOP_COUNT += 1

	# ========= STATE: Start =========
	# Wait for signal that loop has started
	Info.Request("State", {"name":"Start"})

	# happens only in the very first loop iteration
	if LOOP_COUNT == 1:
	
		# Get slides text from Web, then
		list_of_slides_text = Info.Request("Slides")

		# Generate script and append to list
		for slide in list_of_slides_text:
			script = slide
			list_of_scripts.append(script)

	# for subsequent loops, just send the script in the list
	script = list_of_scripts[LOOP_COUNT-1]
	Info.Send("LectureScript", {"text": script})


	# ========= STATE: AnyQuestions =========
	# Wait for state update
	state = Info.Request("State", {"name":"AnyQuestions"})

	# If hands raised, start QnA loop
	while state == "HandsRaised":

		# Wait for student's question from Speech
		question = Info.Request("Question")

		# TODO: Classify question
		question_type = random.choice(["related","operational"])

		if question_type == "related":
			# TODO: For lecture-related questions,
			# generate answer from received question then send to Speech
			answer = "Because blue light is scattered the most"
			Info.Send("Answer", {"text": answer})

		else:
			# TODO: For operational questions, 
			# request for the corresponding action,
			# then generate response and send to Speech
			Action.Request("ChangeVolume", {"cmd":"up"})
			answer = "Got it, I'll speak louder"
			Info.Send("Answer", {"text": answer})

		state = Info.Request("State", {"name":"AnyQuestions", "print":False})

	# When QnA loop ends, proceed


	# ========= STATE: NoiseLevel =========
	# Wait for state update
	state = Info.Request("State", {"name":"NoiseLevel"})

	# If high noise level, get signal from Control to trigger joke or shutup
	# Then send joke/shutup text to Speech. And the loop restarts
	if state == "High":
		signal = Info.Request("TriggerJokeOrShutup")
		# TODO: Generate joke text or shutup text
		if signal == "joke":
			joke = "I'm telling a joke. Laugh. Ha ha."
			Info.Send("Joke", {"text": joke})
		elif signal == "shutup":
			shutup = "Shut up all of you"
			Info.Send("Shutup", {"text": shutup})
		return

	# Else if low noise level, proceed to next state


	# ========= STATE: Attentiveness =========
	# Wait for state update
	state = Info.Request("State", {"name":"Attentiveness"})

	# If not attentive, get signal from Control to trigger joke or quiz
	if state == "NotAttentive":
		signal = Info.Request("TriggerJokeOrQuiz")
		
		# If trigger_joke, send joke to Speech
		if signal == "joke":
			# TODO: generate joke text
			joke = "I'm telling a joke. Laugh. Ha ha."
			Info.Send("Joke", {"text": joke})

		# After joke has been spent, or if trigger_quiz, loop restarts
		return

	# Else if attentive, proceed


	# ========= STATE: NoQuestionsLoop =========
	# Wait for state update from master to check if no_questions_loop has reached counter threshold
	state = Info.Request("State", {"name":"NoQuestionsLoop"})
	
	# If loop counter reached, Control triggers joke or quiz and loop restarts
	if state == "CounterReached":
		signal = Info.Request("TriggerJokeOrQuiz")
		if signal == "joke":
			# TODO: generate joke text
			joke = "I'm telling a joke. Laugh. Ha ha."
			Info.Send("Joke", {"text": joke})
		return

	# Else, restart loop
	return
		

# =================================================

if __name__ == "__main__":
	PepperAPI.init("test")
	while True:
		nlp_main()
