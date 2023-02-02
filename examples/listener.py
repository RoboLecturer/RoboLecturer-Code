#!/usr/bin/env python3

import PepperAPI
from PepperAPI import Info
import rospy

if __name__ == "__main__":
	PepperAPI.init("subteam_node")
	msg = Info.Request("RLTest", {"listen_once": True})
	print(msg)
	rospy.Rate(5).sleep()
	Info.Send("RLTest", {"value":msg})
