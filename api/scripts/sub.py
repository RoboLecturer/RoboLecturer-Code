#!/usr/bin/env python3

import PepperAPI
from PepperAPI import Info

if __name__ == "__main__":
	# Initialise API module
	PepperAPI.init("api_module")

	# Subscribe
	# msg = Info.Request("RLSimpleMsg", {"listen_once":True})
	msg = Info.Request("RLHandLocation", {})
	print("Received message: %s" % str(msg))
