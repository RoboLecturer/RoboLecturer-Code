# coding: utf-8
import PepperAPI
from PepperAPI import Action, Info
from api.msg import CVInfo
from mutagen.flac import FLAC

PepperAPI.init("master")
# msg = Info.Request("LectureScript")
# print(msg)
# slides_text = Info.Request("Slides")
# print(slides_text)

# Info.Send("SimpleMsg", {"value":"hello world"})
path = "/home/user/Downloads/sample.flac"
audio = FLAC(path)
length = audio.info.length
Action.Request("ALAudioPlayer", {
	"path": path, 
	"length":length
	})

# msg = CVInfo()
# msg.x, msg.y, msg.w, msg.h = 560, 360, 0, 0
# msg.frame_width, msg.frame_height = 1080, 720
# msg.score = -1
# Action.Request("Point", {"info": msg})
