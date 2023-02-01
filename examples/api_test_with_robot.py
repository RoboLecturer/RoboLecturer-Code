#!/usr/bin/env python3

"""
Sample script
Example: Speech subteam
Requests for mic input from Pepper, processes text, then sends WAV file to be played
"""
import PepperAPI
from PepperAPI import Action, Info

def main():
	# Initializers the publishers/subscribers you need
	PepperAPI.init()

	# Request for mic input data
	stt_input = Info.Request("ALAudioDevice", [])

	"""
	Processing of input
	"""
	WAV_output = None

	# Send WAV output to be played
	Action.Request("ALAudioPlayer", [WAV_output])
