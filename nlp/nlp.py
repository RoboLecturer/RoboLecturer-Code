import PepperAPI
from PepperAPI import Action, Info
# import random
from tqdm import tqdm 
from pkg import scriptGenerator
from pkg import questionAnswer
from pkg import questionClassifier
from pkg import jokeGenerator
from pkg import descriptionGenerator
from pkg import quizGeneration

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
		class_description = descriptionGenerator.initDict()
		slide_number = 1

		# for each slide, generate script and keyword descriptions
		for slide in tqdm(list_of_slides):
			script = scriptGenerator.createScript(slide, slide_number)
			list_of_scripts.append(script) 

			# SET SLIDE CLASS
			# get slide title 
			title = descriptionGenerator.getTitle(slide)
			# set class
			Slide_instances.append(Slide(slide_number, title, slide, script))

			if slide_number > 2:
				# create question classification content classes and keyword descriptions
				class_description = descriptionGenerator.createDescription(slide,script)
				# create quiz for this slide
				# quiz = quizGeneration.quizGen(script)
				# list_of_quizes.append(list_of_quizes, quiz)
			slide_number += 1

		# Send entrie lecture content to the Speech Processing module for pre-processing
		Info.Send("LectureScript", {"text": list_of_scripts})
		# Send entire quiz list to web
		# Info.Send("Quiz", {"text": list_of_quizes})


	# ========= STATE: AnyQuestions =========
	# Wait for state update
	state = Info.Request("State", {"name":"AnyQuestions"})

	# If hands raised, start QnA loop
	while state == "HandsRaised":

		# Wait for student's question from Speech
		Q.question = Info.Request("Question")

		# Classify question into main type and sub types
		Q.main_type, Q.sub_type = questionClassifier(Q.question,class_description)
		# Q.main_type = "related" 

		if Q.main_type == "related":
			# TODO: POST MVP: High/Low order question classification for different speed model answer generation
			# generate answer from received question then send to Speech
			Q.answer = questionAnswer.answerGen(Q.question)
			response = f"{Q.answer}.. Does that answer your question?"
			Info.Send("Answer", {"text": response})
			# request slide change after sending text to Speech Processing module as text->speech takes time
			for instance in Slide_instances:
				if instance.title == Q.sub_type :
					Info.Request("ChangeSlide", {"cmd":f"{instance.slideNo}"})

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

			elif Q.subtype == "increase speech speed":
				Action.Request("ChangeSpeed", {"cmd":"up"})
				response = "Got it, I'll speed up"
				Info.Send("Answer", {"test": response})

			elif Q.subtype == "decrease speech speed":
				Action.Request("ChangeSpeed", {"cmd":"down"})
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
