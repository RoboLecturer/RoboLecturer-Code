#!/usr/bin/env python3

import PepperAPI
from PepperAPI import Info, Publisher

if __name__ == '__main__':
	PepperAPI.init("subteam_node")
	Info.Send("RLTest", {"value":"Hello World"})
	Info.Request("RLTest", {"listen_once":True})
