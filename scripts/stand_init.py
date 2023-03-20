from naoqi import ALProxy, ALBroker
posture = ALProxy("ALRobotPosture", "192.168.0.110", 9559)
posture.applyPosture("StandInit",0.2)
