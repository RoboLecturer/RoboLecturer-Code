from naoqi import ALProxy
tracker = ALProxy("ALTracker", "192.168.0.110", 9559)
frame = 0 # Torso=0, World=1, Robot=2
max_speed = 0.5
tracker.pointAt("LArm", [1, 0.7, 0.3], frame, max_speed)


