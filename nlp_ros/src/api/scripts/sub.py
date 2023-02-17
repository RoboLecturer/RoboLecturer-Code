#!/usr/bin/env python3

import PepperAPI
from PepperAPI import Info
import time, sys

def simulate_cv_module():
	# Waits for trigger_hand_detection, sends hand_info,
	# then waits for trigger_attentiveness_detection, then sends
	# trigger_noise_detection to Speech
	
	print("\nWaiting for trigger_hand_detection...")
	if not Info.Request("TriggerHandDetection"):
		return

	time.sleep(2)
	print("\nSending hand info...")
	Info.Send("RaisedHandInfo", {
        "bounding_box": (1.0, 2.0, 3.0, 4.0),
        "frame_res": (1920, 1080),
        "confidence_score": 0.96
    })

	print("\nWaiting for trigger_attentiveness_detection...")
	if not Info.Request("TriggerAttentivenessDetection"):
		return

	time.sleep(2)
	print("\nSend trigger_noise_detection...")
	Info.Send("TriggerNoiseDetection")

	return

if __name__ == "__main__":
	## Initialise API module
	PepperAPI.init("api_module")

	counter = 0
	try:
		while True:
			print("\nTrial %d" % counter)
			simulate_cv_module()
			counter += 1
	except KeyboardInterrupt:
		sys.exit(0)

