# coding: utf-8
import PepperAPI
from PepperAPI import Action, Info
from api.msg import CVInfo

PepperAPI.init("master")
msg = CVInfo()
msg.x, msg.y, msg.w, msg.h = 740, 0, 0, 0
msg.frame_width, msg.frame_height = 1280, 720
msg.score = -1
Action.Request("Point", {"info": msg})
