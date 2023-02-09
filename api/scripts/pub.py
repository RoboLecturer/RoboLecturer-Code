#!/usr/bin/env python3

import PepperAPI
from PepperAPI import Action, Info
import cv2

if __name__ == "__main__":
	# Initialise API module
	PepperAPI.init("api_module")

	# Publish info
	# Info.Send("RLSimpleMsg", {"value": "Hello World"})
	# Info.Send("RLHandLocation", {"value":[1.23, 5.42]})
	
	# Send image
	# img = cv2.imread("image.jpg")
	# img_blur = cv2.GaussianBlur(img,(3,3), sigmaX=0, sigmaY=0)
	# sobel = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5)
	# Info.Send("RLImage", {"value": sobel})

	# Request to play audio	
	Action.Request("ALTextToSpeech", {"value":"Hello World"})
	# Action.Request("ALAudioPlayer", {"path":"/home/user/sample.mp3"})
