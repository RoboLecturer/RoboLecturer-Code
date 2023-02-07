#!/usr/bin/env python3

import PepperAPI
from PepperAPI import Info
import rospy

if __name__ == "__main__":
	# Initialise API module
	PepperAPI.init("api_module")

	# Publish info
	# Info.Send("RLSimpleMsg", {"value": "Hello World"})
	Info.Send("RLHandLocation", {"value":[1.23, 5.42]})


