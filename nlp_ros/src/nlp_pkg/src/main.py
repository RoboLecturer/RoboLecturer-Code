#!/usr/bin/env python3

"""
Sample script
Example: NLP subteam
Receives data from Webserver team, processes it, then sends the text to be said to the Speech team
Sends quiz results data from Webserver subteam to Speech subteam
"""

import PepperAPI
from PepperAPI import Info

def main():
	# Initializers the publishers/subscribers you need
	PepperAPI.init()

	# Request for data from Webserver
	# API name here can be defined by you
	# RL is just short for RoboLecturer, to be differentiated with Naoqi's "AL" APIs
	data = Info.Request("RLWebData", [])

	"""
	Processing of webserver data
	"""
	processed_text = []

	# Send text to Speech team
	Info.Send("RLText", [processed_text])