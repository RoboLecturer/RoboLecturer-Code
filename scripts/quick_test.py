from naoqi import ALProxy, ALBroker
import time

dev = ALProxy("ALRobotPosture", "192.168.0.104", 9559)
dev.applyPosture("StandInit",0.2)
# dev.pointAt("LArm", [1,1.5,0.3], 0, 0.5)

