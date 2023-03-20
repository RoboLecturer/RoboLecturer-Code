# coding: utf-8
import PepperAPI
from PepperAPI import Action, Info
from api.msg import CVInfo

PepperAPI.init("master")
lst = ["a","b"]
Info.Send("NumScripts", {"value":len(lst)})
for i in lst:
	Info.Send("LectureScript", {"text":i})
# slides_text = Info.Request("Slides")
# print(slides_text)

# Info.Send("SimpleMsg", {"value":"hello world"})
# Action.Request("ALAudioPlayer", {"path":"/home/user/sample.flac"})

# msg = CVInfo()
# msg.x, msg.y, msg.w, msg.h = 560, 360, 0, 0
# msg.frame_width, msg.frame_height = 1080, 720
# msg.score = -1
# Action.Request("Point", {"info": msg})
