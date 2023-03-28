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
from pkg.chat import chat_model, pine
import requests
import sys

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
lecture_title = ""
list_of_questions = [] # store questions for post-evaluation (higher-order/lower-order)

class convHistory:
	def __init__(self, context):
		self.context = context
		self.history = list()
		self.salience = ""
		self.anticipation = ""

		self.history.append({'role': 'system', 'content': self.context}) # element of list for personal context and salience
		self.history.append({'role': 'assistant', 'content': ""}) # element of list for relevant lecture script
		self.history.append({'role': 'assistant', 'content': ""}) # element of list for additional relevant textbook extracts
	
	def getSalience(self, convoContext):
		"""generate the salience of the current conversation
		@params: convoContext: list|string - list of the conversations 
		"""

		# hist = self.flatternConvo(convoContext) NO NEED FOR THIS ANYMORE BECAUSE THE INPUT IS A SINGLE STRING IN CORRECT FORMAT
		query_salience = f"Given the following chat log, write a brief summary of only the most salient points of the conversation:\n\n {convoContext}"
		
		self.salience = chat_model.getResponse(query_salience)
	
	def getAnticipation(self):
		"""Generate the anticipation of the users needs"""

		hist = self.flatternConvo(self.history)
		
		query_anticipate = f"Given the following chat log, infer the user's actual information needs. Attempt to anticipate what the uder truly needs even if the user does not fully understand it uet themselves, or is asking the wrong questions.\n\n {hist}"
		
		self.anticipation = chat_model.getResponse(query_anticipate)
	
	
	def updateHistoryContext(self):
		"""update Peppers personal and historical context"""

		self.history[0]['content'] = self.context + "I am in the middle of a conversation: %s. I anticipate the user needs: %s. I will do my best to fulfill my objectives." % (self.salience, self.anticipation)

	def updateSlideContext(self, script, context):
		"""update the conversation history with the script from the slide relevant to the query
		@params: 
			script: list|[string] - lecture script
			context: list|[string] - extra lecture material
		"""
		# flatten the incoming list - script
		script = " ".join(script)
		script = "\n".join(context)
		self.history[1]['content'] = script
		self.history[2]['content'] = "This is extra relevant context to the question, according to the textbooks: %s" % context
	
	def flatternConvo(self, history):
		"""create a single string out of a list of conversations
		@params: 
			history: dict{}|string - dictionary of conversations, context and scripts
		"""
		convo=""
		for i in history:
			convo += '%s: %s\m' % (i['role'].upper(), i['content'])
		return convo.strip()

	def update(self, script, context):
		"""update all of the conversation facets ready for query processing"""
		# update the history context
		self.updateHistoryContext()
		# update the slide context
		self.updateSlideContext(script, context)
		

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

	global lecture_title, list_of_scripts, LOOP_COUNT, Q, class_description, Slide_instances, list_of_quizes, list_of_questions

	vdb = pine.init_pinecone()
	
	conversation = convHistory("I am an AI Teacher and lecturer. I have 5 goals: teach my students the lesson plan I am given, answer their questions to clear up areas of ambiguity, ask them questions to gauge understanding  and quiz them, maintain order in the classroom, and be ultimately helpful.")

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

			# get slide title 
			title = dg.getTitle(slide)
			if slide_number == 0:
				lecture_title=title
			# set class
			Slide_instances.append(Slide(slide_number, title, slide, script))
			# create slide metadata for storage in Pinecone
			metadata = pine.createSlideMetadata(slide_number, script, title)
			# store slide script and metadata in Pinecone, under namespace "scripts"
			pine.populatePinecone(script, "scripts", metadata, vdb)

			if slide_number > 0:
				# create question classification content classes and keyword descriptions
				class_description = dg.createDescription(slide,script,class_description)
				# create quiz for this slide
				# quiz = qg.quizGen(script)
				# url = "http://192.168.0.101:3000/saveQuiz"
				# requests.post(url, json = quiz)
				# list_of_quizes.append(list_of_quizes, quiz)
			slide_number += 1

		# Send entrie lecture content to the Speech Processing module for pre-processing
		Info.Send("NumScripts", {"value": slide_number})
		for script in list_of_scripts:
			Info.Send("LectureScript", {"text": script})
		# Send entire quiz list to web
		# Info.Send("Quiz", {"text": list_of_quizes})

	if LOOP_COUNT == len(list_of_scripts) + 1:
		sys.exit()

	# ========= STATE: AnyQuestions =========
	# Wait for state update
	state = Info.Request("State", {"name":"AnyQuestions"})

	c = 1
	# If hands raised, start QnA loop
	while state == "HandsRaised":

		# Wait for student's question from Speech
		Q.question = Info.Request("Question")
		# coherent = qc.is_coherent(Q.question, lecture_title)
		coherent = True

		if Q.question == None or Q.question == "" or Q.question == "None":
			Q.main_type = "no question"
		elif coherent == False:
			Q.main_type = "non-coherent"
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
					if instance.title == Q.sub_type:
						# scriptContent = instance.scriptContent
						title = instance.title
						slide = instance.slideNo

				# get conversations relvant to the query
				convoContext = pine.queryPinecone(Q.question, vdb, "conversation", title) 
				# generate the salience and anticipation
				conversation.getSalience(convoContext)
				conversation.getAnticipation()
				# generate the content context
				contentContext = pine.queryPinecone(Q.question, vdb, "textbook", title) # script namesapce includes lecture slides and textbook contents
				lectureScript = pine.queryPinecone(Q.question, vdb, "script", title)
				conversation.update(contentContext, lectureScript) # update the convo history with the salience, anticipation and content  

				# generate answer from received question then send to Speech
				Q.answer = qa.answerGen(Q.question, conversation.history, 0)

				# create QnA metadata for storage in Pinecone
				metadata = pine.createConvoMetadata(title, f"{Q.question}", f"{Q.answer}") 
				pine.populatePinecone(f"{Q.question}, {Q.answer}", "conversation", metadata, vdb)

				response = f"{Q.answer}.. Does that answer your question?"
				Info.Send("Answer", {"text": response})
				# request slide change after sending text to Speech Processing module as text->speech takes time
				Info.Request("ChangeSlide", {"cmd":f"{slide}"})

				# append question to list for post-evaluation
				list_of_questions.append(Q.question + "\n")


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
					Info.Send("Answer", {"text": response})

				elif Q.sub_type == "go to specific slide number":
					# how do we extract the slide number that they want?
					# TODO: how to get the specific slide
					slide_no = 5
					Info.Request("ChangeSlide", {"cmd": f"goto|{slide_no}"})
					response = "Got it, I'll go to that slide"
					Info.Send("Answer", {"text": response})

			elif Q.main_type == "no question" or Q.main_type == "non-coherent":
				# don't do anything
				Info.Send("Answer", {"text": "I didn't get that. Can you please repeat the question?"})

			elif Q.main_type == "Finished":
				Info.Send("Answer", {"text": "Ok. let's move on"})

			else:
				# if question is non-relevant then respond as such
				response = "I don't think your question relates to the content. Should we move on, or do you have a relevant question?"
				Info.Send("Answer", {"text": response})

		print("Loop no. " + str(c))
		c += 1
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
