import PepperAPI
from PepperAPI import Action, Info
from api.msg import CVInfo
import time

PepperAPI.init("master")

lst = [[100,600],[540,50],[740,50],[1180,600]] # left, right

for x,y in lst:
	msg = CVInfo()
	msg.x, msg.y, msg.w, msg.h = x,y, 0, 0
	msg.frame_width, msg.frame_height = 1280, 720
	msg.score = -1
	Action.Request("Point", {"info": msg})
	time.sleep(1)
