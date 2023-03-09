from naoqi import ALProxy

ap = ALProxy("ALAudioPlayer", "192.168.0.104", 9559)
ap.stopAll()
