import os
from naoqi import ALProxy
import cv2
import numpy as np
import vision_definitions
import Image

vid = ALProxy("ALVideoDevice", os.environ["NAO_IP"], 9559)
resolution = vision_definitions.kQVGA
colorSpace = vision_definitions.kRGBColorSpace
fps = 10
subscriberID = vid.subscribe("subscriberID", resolution, colorSpace, fps)
num = 0
while True:
	img = vid.getImageRemote(subscriberID)
	imgWidth, imgHeight = img[0], img[1]
	array = img[6]
	im = Image.frombytes("RGB", (imgWidth, imgHeight), array)
	frame = np.array(im)[:,:,::-1] # convert to BGR
	frame = cv2.resize(frame, (640,480), interpolation=cv2.INTER_AREA)
	cv2.line(frame, (320,0), (320, 480), (255,0,0), 2)
	cv2.imshow("img", frame)
	key = cv2.waitKey(1) & 0xFF
	if key == ord('q'):
		break
	if key == ord('p'):
		cv2.imwrite("grid%d.jpg" % num, frame)
		print("Image saved to grid%d.jpg." % num)
		num += 1

cv2.destroyAllWindows()