import os
from naoqi import ALProxy
import almath

ROBOT_IP = os.environ["NAO_IP"]
motion = ALProxy("ALMotion", ROBOT_IP, 9559)
motion.wakeUp()
posture = ALProxy("ALRobotPosture", ROBOT_IP, 9559)
posture.applyPosture("StandInit",0.5)
names = "HeadPitch"
angleLists = 10.0*almath.TO_RAD
motion.angleInterpolation(names, angleLists, 2.0, True)
