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
from pkg.chat import chat_model 

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
list_of_questions = [] # store questions for post-evaluation (higher-order/lower-order)

class covnHistory:
	def __init__(self, context, maxLength = 30):
		self.context = context
		self.maxLength = maxLength
		self.history = list()
		self.salience = ""
		self.anticipation = ""
		self.embedding = list()

		self.history.append({'role': 'system', 'content': self.Context})
		self.history.append({'role': 'assistant', 'content': ""})

	def customAppend(self, myList, item):
		"""function to append to the list, if its bellow the size limit. If not, then shoft elements to the left by one and add new item"""
		def shift_left(lst):
			temp = lst[0]
			for i in range(1, len(lst)):
				lst[i-1] = lst[i]
			lst[-1] = temp
			return lst

		length = len(myList)
		if length == self.maxLength:
			myList = shift_left(myList)
			myList[-1] = item 
		else:
			myList.append(item)
		return myList
	
	def getSalience(self):
		"""generate the salience of the current conversation"""

		hist = self.flatternConvo(self.history)
		
		query_salience = f"Given the following chat log, write a brief summary of only the most salient points of the conversation:\n\n {hist}"
		
		self.salience = chat_model.getResponse(query_salience)
	
	def getAnticipation(self):
		"""Generate the anticipation of the users needs"""

		hist = self.flatternConvo(self.history)
		
		query_anticipate = f"Given the following chat log, infer the user's actual information needs. Attempt to anticipate what the uder truly needs even if the user does not fully understand it uet themselves, or is asking the wrong questions.\n\n {hist}"
		
		self.anticipation = chat_model.getResponse(query_anticipate)
	
	
	def updateHistoryContext(self):
		"""update Peppers personal and historical context"""

		self.history[0]['content'] = self.Context + "I am in the middle of a conversation: %s. I anticipate the user needs: %s. I will do my best to fulfill my objectives." % (self.salience, self.anticipation)

	def updateSlideContext(self, script):
		"""update the conversation history with the script from the slide relevant to the query"""

		self.history[1]['content'] = script
	
	def flatternConvo(self, conversation):
		convo=""
		for i in conversation:
			convo += '%s: %s\m' % (i['role'].upper(), i['content'])
		return convo.strip()
	
	def updateHistory(self, query, response):
		self.customAppend(self.history, {'role': 'user', 'content': query})
		self.customAppend(self.history, {'role': 'assistant', 'content': response})

	def update(self, script):
		"""update all of the conversation facets ready for query processing"""
		# update the salience
		self.getSalience()
		# update the anticipation
		self.getAnticipation()
		# update the history context
		self.updateHistoryContext()
		# update the slide context
		self.updateSlideContext(script)

	def getEmbedding(self, pc_query):
		"""get PineCone query embedding"""
		self.embedding = chat_model.getEmbedding(pc_query)
		



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

	global list_of_scripts, LOOP_COUNT, Q, class_description, Slide_instances, list_of_quizes, list_of_questions

	conversation = covnHistory("I am an AI Teacher and lecturer. I have 5 goals: teach my students the lesson plan I am given, answer their questions to clear up areas of ambiguity, ask them questions to gauge understanding  and quiz them, maintain order in the classroom, and be ultimately helpful.", 30)

	LOOP_COUNT += 1
	# ========= STATE: Start =========
	# Wait for signal that loop has started
	Info.Request("State", {"name":"Start"})

	if LOOP_COUNT == 1: # happens only in the very first loop

		# Get all slides from web
		list_of_slides = Info.Request("Slides")
		# initialise the class_descriptions dictionary with operational keys
		class_description = dg.initDict()
		slide_number = 1

		# for each slide, generate script and keyword descriptions
		for slide in tqdm(list_of_slides):
			script = sg.createScript(slide, slide_number)
			list_of_scripts.append(script)

			# SET SLIDE CLASS
			# get slide title 
			title = dg.getTitle(slide)
			# set class
			Slide_instances.append(Slide(slide_number, title, slide, script))

			if slide_number > 2:
				# create question classification content classes and keyword descriptions
				class_description = dg.createDescription(slide,script,class_description)
				# create quiz for this slide
				# quiz = quizGeneration.quizGen(script)
				# list_of_quizes.append(list_of_quizes, quiz)
			slide_number += 1

		# Send entrie lecture content to the Speech Processing module for pre-processing
		Info.Send("NumScripts", {"value": slide_number})
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
		Q.main_type, Q.sub_type = qc(Q.question,class_description)
		# Q.main_type = "related" 

		if Q.main_type == "related":
			for instance in Slide_instances:
				if instance.title == Q.sub_type :
					scriptContent = instance.scriptContent
					title = instance.title
					slide = instance.slideNo

			# prepare the conversation hisotry
			conversation.update(scriptContent)

			# generate answer from received question then send to Speech
			Q.answer = qa.answerGen(Q.question, conversation.history, 0)

			# update the conversation
			conversation.updateHistory(Q.question, Q.answer)

			response = f"{Q.answer}.. Does that answer your question?"
			Info.Send("Answer", {"text": response})
			# request slide change after sending text to Speech Processing module as text->speech takes time
			Info.Request("ChangeSlide", {"cmd":f"{slide}"})

			# append question to list for post-evaluation
			list_of_questions.append(Q.question + "\n")


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
				response = "Got it, I'll speed up"
				Info.Send("Answer", {"test": response})

			elif Q.subtype == "decrease speech speed":
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
			joke = jg.genJoke("noiseHigh")
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
	PepperAPI.init("nlp")
	try:
		while True:
			nlp_main()

	# when code is terminated, store questions asked for post-evaluation
	except KeyboardInterrupt:
		if len(list_of_questions):
			f = open("higher_order_lower_order.txt", "w")
			f.writelines(list_of_questions)
			f.close()
			print("Questions stored to file.")
