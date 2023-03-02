import PepperAPI
from PepperAPI import Action, Info
import random 
from nlp import scriptGenerator
from nlp import questionAnswer
from nlp import questionClassifier
from nlp import jokeGenerator

list_of_scripts = []
LOOP_COUNT = 0
class Q:
	question = ""
	main_type = ""
	sub_type = ""
	answer = ""

def nlp_main():

	global list_of_scripts, LOOP_COUNT, Q

	LOOP_COUNT += 1
	# ========= STATE: Start =========
	# Wait for signal that loop has started
	Info.Request("State", {"name":"Start"})

	if LOOP_COUNT == 1: # happens only in the very first loop
		# Get slides text from Web, then
		list_of_slides = Info.Request("Slides")
		slide_number = 1
		# Web can send the "DONE" string to indicate that there are no more slides
		for slide in list_of_slides:
			script = scriptGenerator.createScript(slide, slide_number)
			list_of_scripts.append(script) 
			if slide_number > 2:
				# TODO: create question classification content classes and keyword descriptions
				continue
			slide_number += 1
		# Send entrie lecture content to the Speech Processing module for pre-processing
		Info.Send("LectureScript", {"text": list_of_scripts})


	# ========= STATE: AnyQuestions =========
	# Wait for state update
	state = Info.Request("State", {"name":"AnyQuestions"})

	# If hands raised, start QnA loop
	while state == "HandsRaised":

		# Wait for student's question from Speech
		Q.question = Info.Request("Question")

		# Classify question into main type and sub types
		Q.main_type, Q.sub_type = questionClassifier(Q.question)
		# Q.main_type = "related" 

		if Q.main_type == "related":
			# generate answer from received question then send to Speech
			Q.answer = questionAnswer.answerGen(Q.question)
			response = f"{Q.answer}.. Does that answer your question?"
			Info.Send("Answer", {"text": response})

		elif Q.main_type == "operational":
			# if the quesiton is operational, check the command type
			if Q.subtype == "increase speech volume":
				Action.Request("ChangeVolume", {"cmd":"up"})
				response = "Got it, I'll speak louder"
				Info.Send("Answer", {"text": response})
			elif Q.subtype == "decrease speech volume":
				Action.Request("ChangeVolume", {"cmd":"down"})
				response = "Got it, I'll speak quieter"
				Info.Send("Answer", {"text": response})
			# TODO: ADD the rest of the operational call when they are implemented

		else:
			# if question is non-relevant then respond as such
			response = "Your question doesn't relate to the lecture content, lets get back on track"
			Info.Send("Answer", {"text": response})

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
			joke = jokeGenerator.genJoke("noiseHigh")
			Info.Send("Joke", {"text": joke})
		elif signal == "shutup":
			response = "Come on guys. Please try and concentrate, this is some interesting shit i'm teaching here"
			Info.Send("Shutup", {"text": response})
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
			# generate joke text
			joke = jokeGenerator.genJoke("attentionLow")
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
			# generate joke text
			joke = jokeGenerator.genJoke("")
			Info.Send("Joke", {"text": joke})
		
		# After joke has been sent, or if trigger_quiz, loop restarts 
		return

	# Else, restart loop
	return
		

# =================================================

if __name__ == "__main__":
	PepperAPI.init("test")
	while True:
		nlp_main()
