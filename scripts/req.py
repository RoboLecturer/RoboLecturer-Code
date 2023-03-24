# coding: utf-8
import PepperAPI
from PepperAPI import Action, Info
from mutagen.mp3 import MP3

PepperAPI.init("master")
# Info.Request("SimpleMsg")

# Info.Request("LectureScript")
# Info.Request("Slides")

path = "/home/user/Downloads/statquest.mp3"
audio = MP3(path)
length = audio.info.length
Action.Request("ALAudioPlayer", {
	"path": path, 
	"length":length
	})
