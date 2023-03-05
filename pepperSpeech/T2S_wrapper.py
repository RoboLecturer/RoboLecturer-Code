#!/usr/bin/env python3
import PepperAPI
from PepperAPI import Action
from text2speech import *

if __name__ == "__main__":
	# initialize the API
	PepperAPI.init("speech_node")
	
	INPUT_PATH = "text.txt"

	with open(INPUT_PATH, 'r') as file:
		txt = file.read().rstrip()
		
	filepath = runT2S(txt)
	Action.Request("ALAudioPlayer", {"path": filepath})