# Note that you might not have this module depending on your distribution
from __future__ import print_function
import os
import sys
import time
from naoqi import ALProxy

IP = os.environ["NAO_IP"]
PORT = 9559

try:
 	proxy = ALProxy("ALPhotoCapture", IP, PORT)
except Exception as e:
	print(str(e))
	exit(1)

proxy.setResolution(2) # Set resolution to VGA (640 x 480)
proxy.setPictureFormat("jpg")
for i in range(10):
	print("Taking...")
	time.sleep(3)
	proxy.takePictures(1, "/home/nao/recordings/cameras", "grid%d" % i)
	print("Done")
# # We'll save a 5 second video record in /home/nao/recordings/cameras/
# proxy.startRecording("/home/nao/recordings/cameras", "test")
# print "Video record started."
# time.sleep(5)
# videoInfo = proxy.stopRecording()
# print "Video was saved on the robot: ", videoInfo[1]
# print "Total number of frames: ", videoInfo[0]
