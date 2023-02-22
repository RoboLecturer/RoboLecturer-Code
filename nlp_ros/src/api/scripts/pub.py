#!/usr/bin/env python3

import PepperAPI
from PepperAPI import Action, Info
import time, sys

def main():
	
	time.sleep(2)
	print("\nSending trigger_hand_detection...")
	Info.Send("TriggerHandDetection")

	print("\nReceiving hand info...")
	hand1 = Info.Request("RaisedHandInfo")
	print(hand1)

	time.sleep(2)
	print("\nSending trigger_attentiveness_detection...")
	Info.Send("TriggerAttentivenessDetection")

	print("\nWaiting for trigger_noise_detection...")
	Info.Request("TriggerNoiseDetection")


if __name__ == "__main__":
	## Initialise API module
	PepperAPI.init("api_module")

	counter = 0
	try:
		while True:
			print("\nTrial %d" % counter)
			main()
			counter += 1
	except KeyboardInterrupt:
		sys.exit(0)

	## Send image
	# img = cv2.imread("image.jpg")
	# img_blur = cv2.GaussianBlur(img,(3,3), sigmaX=0, sigmaY=0)
	# sobel = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5)
	# Info.Send("Image", {"value": sobel})

	## Action requests
	# Action.Request("ALTextToSpeech", {"value":"Hello World"})
	# Action.Request("ALAudioPlayer", {"path":"/home/user/sample.mp3"})
