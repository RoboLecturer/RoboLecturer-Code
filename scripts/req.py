# coding: utf-8
import PepperAPI
from PepperAPI import Action, Info
from api.msg import CVInfo

PepperAPI.init("master")
msg = CVInfo()
msg.x = 837.00
msg.y = 438.00
msg.w = 100
msg.h = 100
msg.

info = {
	"bounding_box": (837.00, 438.00, 100, 100),
	"frame_res": (1920,1080),
	"confidence_score": -1
}
hands_info_list = [info]
# Info.Request("SimpleMsg")
# Info.Send("SimpleMsg", {"value":"1"})
# Action.Request("ALAudioPlayer", {"path":"/home/user/sample.flac"})
Action.Request("Point", {"info": info})
# Action.Request("ALAudioPlayer", {"file":"output0.wav"})
# text = "hello world"
# Action.Request("ALTextToSpeech", {"value":text})
# Info.Send("State", {"AnyQuestions":"HandsRaised"})
