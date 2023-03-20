from naoqi import ALProxy
import cv2
import numpy as np
import vision_definitions
import Image

vid = ALProxy("ALVideoDevice", "192.168.0.110", 9559)
resolution = vision_definitions.kQQVGA
colorSpace = vision_definitions.kRGBColorSpace
fps = 10
subscriberID = vid.subscribe("subscriberID", resolution, colorSpace, fps)
while True:
	img = vid.getImageRemote(subscriberID)
	imgWidth, imgHeight = img[0], img[1]
	array = img[6]
	im = Image.frombytes("RGB", (imgWidth, imgHeight), array)
	mat = np.array(im)[:,:,::-1] # convert to BGR
	cv2.imshow("img", mat)
	key = cv2.waitKey(1) & 0xFF
	if key == ord('q'):
		break
cv2.destroyAllWindows()
