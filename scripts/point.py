from naoqi import ALProxy
import os
tracker = ALProxy("ALTracker", os.environ["NAO_IP"], 9559)
frame = 0 # Torso=0, World=1, Robot=2
max_speed = 0.5
tracker.pointAt("LArm", [1, 0.7, 0.3], frame, max_speed)


