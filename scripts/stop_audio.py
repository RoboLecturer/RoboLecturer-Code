import os
from naoqi import ALProxy

ap = ALProxy("ALAudioPlayer", os.environ["NAO_IP"], 9559)
ap.stopAll()
