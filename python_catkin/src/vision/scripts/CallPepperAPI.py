#!/usr/bin/env python3

import PepperAPI
from PepperAPI import Action, Info

"""
Request for STT, process the input, then request for TTS to play the processed message
"""

def main():
	# Initializers the publishers/subscribers you need
	PepperAPI.init()

	# Request for vision system to be on
	stt_input = Info.Request("ALVisionRecognition", [])

	"""
	Processing of input
	"""
	photo_output = None

	# Send photo output to be captured
	Action.Request("ALPhotoCapture", [photo_output])