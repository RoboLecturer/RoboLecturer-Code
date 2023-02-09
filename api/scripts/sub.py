#!/usr/bin/env python3

import PepperAPI
from PepperAPI import Info
import cv2

if __name__ == "__main__":
	# Initialise API module
	PepperAPI.init("api_module")

	# Subscribe
	msg = Info.Request("RLSimpleMsg", {"listen_once":True})
	print("Received message: %s" % str(msg))
	# msg = Info.Request("RLHandLocation", {})

	# img = Info.Request("RLImage", {})
	# cv2.imwrite("edges.jpg", img)
