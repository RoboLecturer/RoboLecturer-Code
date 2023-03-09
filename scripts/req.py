# coding: utf-8
import PepperAPI
from PepperAPI import Action, Info

PepperAPI.init("master")
# Info.Request("SimpleMsg")
# Info.Send("SimpleMsg", {"value":"1"})
# Action.Request("ALAudioPlayer", {"path":"/home/user/sample.flac"})
Action.Request("ALAudioPlayer", {"file":"output0.wav"})
# text = "hello world"
# Action.Request("ALTextToSpeech", {"value":text})
# Info.Send("State", {"AnyQuestions":"HandsRaised"})
