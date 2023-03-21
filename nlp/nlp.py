import PepperAPI
from PepperAPI import Action, Info
# import random
from tqdm import tqdm 
from pkg import scriptGenerator as sg
from pkg import questionAnswer as qa
from pkg import questionClassifier as qc
from pkg import jokeGenerator as jg
from pkg import descriptionGenerator as dg
from pkg import quizGeneration as qg
import requests

#############################################################################
# TODO: How do we extrct the slide number for a specific slide request?
#  		If the slide request is based on the content of that slide... how 
# 		Do we detemine the slide number that we need.... antoher 
# 		classification?
#############################################################################

list_of_scripts = []
LOOP_COUNT = 0
class_description = {}
Slide_instances = []
list_of_quizes = []
class Q:
	question = ""
	main_type = ""
	sub_type = ""
	answer = ""

class Slide:
	# MAYBE THIS IS BETTER DONE AS DICTIIONARY???????
	def __init__(self, slide_number, slide_title, slide_contents, script_contents):
		self.title = slide_title
		self.slideNo = slide_number
		self.slideContent = slide_contents
		self.scriptContent = script_contents




def nlp_main():

	global list_of_scripts, LOOP_COUNT, Q, class_description, Slide_instances, list_of_quizes

	LOOP_COUNT += 1
	# ========= STATE: Start =========
	# Wait for signal that loop has started
	Info.Request("State", {"name":"Start"})

	if LOOP_COUNT == 1: # happens only in the very first loop

		# Get all slides from web
		list_of_slides = Info.Request("Slides")
		# initialise the class_descriptions dictionary with operational keys
		class_description = dg.initDict()
		slide_number = 0

		# for each slide, generate script and keyword descriptions
		for slide in tqdm(list_of_slides):
			script = sg.createScript(slide, slide_number)
			list_of_scripts.append(script) 

			# SET SLIDE CLASS
			# get slide title 
			title = dg.getTitle(slide)
			# set class
			Slide_instances.append(Slide(slide_number, title, slide, script))

			if slide_number > 1:
				# create question classification content classes and keyword descriptions
				class_description = dg.createDescription(slide,script,class_description)
				# create quiz for this slide
				quiz = qg.quizGen(script)
				url = "localhost:3000/saveQuiz"
				requests.post(url, json = quiz)
				# list_of_quizes.append(list_of_quizes, quiz)
			slide_number += 1

		# Send entrie lecture content to the Speech Processing module for pre-processing
		Info.Send("NumScripts", {"value": slide_number})
		for script in list_of_scripts:
			Info.Send("LectureScript", {"text": script})
		# Send entire quiz list to web
		# Info.Send("Quiz", {"text": list_of_quizes})


	# ========= STATE: AnyQuestions =========
	# Wait for state update
	state = Info.Request("State", {"name":"AnyQuestions"})

	# If hands raised, start QnA loop
	while state == "HandsRaised":

		# Wait for student's question from Speech
		Q.question = Info.Request("Question")

		if Q.question == None:
			Q.main_type = "no question"
		else:
			# Classify question into main type and sub types
			Q.main_type, Q.sub_type = qc.classify_question(Q.question,class_description)
			print(f"question main type: {Q.main_type} - question sub type: {Q.sub_type}")
		# Q.main_type = "related" 

		# TODO: if main type is finished, then send done 
		if Q.main_type == "finished":
			Info.Send("StudentDone", {"value":1})
		else: 
			Info.Send("StudentDone", {"value":0})
		

			if Q.main_type == "related":
				for instance in Slide_instances:
					if instance.title == Q.sub_type :
						scriptContent = instance.scriptContent
						title = instance.title
						slide = instance.slideNo

				# generate answer from received question then send to Speech
				Q.answer = qa.answerGen(Q.question, scriptContent, title)

				response = f"{Q.answer}.. Does that answer your question?"
				Info.Send("Answer", {"text": response})
				# request slide change after sending text to Speech Processing module as text->speech takes time
				Info.Request("ChangeSlide", {"cmd":f"{slide}"})

			elif Q.main_type == "operational":
				# if the quesiton is operational, check the command type
				if Q.sub_type == "increase speech volume":
					Action.Request("ChangeVolume", {"cmd":"up"})
					response = "Got it, I'll speak louder"
					Info.Send("Answer", {"text": response})

				elif Q.sub_type == "decrease speech volume":
					Action.Request("ChangeVolume", {"cmd":"down"})
					response = "Got it, I'll speak quieter"
					Info.Send("Answer", {"text": response})
				# TODO: ADD the rest of the operational call when they are implemented

				elif Q.sub_type == "increase speech speed":
					response = "Got it, I'll speed up"
					Info.Send("Answer", {"test": response})

				elif Q.sub_type == "decrease speech speed":
					response = "Got it, I'll slow down"
					Info.Send("Answer", {"test": response})

				elif Q.sub_type == "go to next slide":
					Info.Request("ChangeSlide", {"cmd": "increment|0"})
					response = "Got it, I'll go to the next slide"
					Info.Send("Answer", {"text": response})

				elif Q.sub_type == "go to previous slide":
					Info.Request("ChangeSlide", {"cmd": "decrement|0"})
					response = "Got it, I'll go to the previous slide"
					Info.Send("Anwer", {"text": response})

				elif Q.sub_type == "go to specific slide number":
					# how do we extract the slide number that they want?
					# TODO: how to get the specific slide
					slide_no = 5
					Info.Request("ChangeSlide", {"cmd": f"goto|{slide_no}"})
					response = "Got it, I'll go to that slide"
					Info.Send("Answer", {"text": response})

			elif Q.main_type == "no question":
				# don't do anything
				Info.Send("Answer", {"text": " "})
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
		# Generate joke text or shutup text
		if signal == "joke":
			joke = jg.genJoke("noiseHigh")
			Info.Send("Joke", {"text": f"Hey everyone. I've got a joke for you... {joke}"})
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
			joke = jg.genJoke("attentionLow")
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
			joke = jg.genJoke("")
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
