# coding: utf-8
import PepperAPI
from PepperAPI import Action, Info
from api.msg import CVInfo

PepperAPI.init("master")
# slides_text = Info.Request("Slides")
# print(slides_text)

Info.Send("SimpleMsg", {"value":"hello world"})
