#!/usr/bin/env
import numpy as np
import cv2
from sys import argv


try:
    sampling = argv[1]
except Exception:
    print("Sampling argument not provided")
    sampling = False

cap = cv2.VideoCapture(0)
cap.set(3, 840)
cap.set(4, 840)
face_detector = cv2.CascadeClassifier("face_detection.xml") # loading the pre-trained face detection model from OpenCV.
iter = 0
engagement = 0.69 # for now an arbitrary number. In the future, it will come from the classification model.

def sample(faces, iter):
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
        roi_color = img[y+8:y+h - 2, x+2:x+w-2] # these numbers were chosen so that the bounding boxes and text artifacts are removed from a face data.
        cv2.imwrite(f"{x}_{y}_{iter}_face.jpg", roi_color)

while True:
    ret, frame = cap.read()
    img = frame#cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # to make frames gray.
    faces = face_detector.detectMultiScale(img, scaleFactor=1.2, minNeighbors=5, minSize=(64, 64))
    
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
        roi_color = img[y:y+h, x:x+w]
        cv2.putText(img, f"Engagement: {engagement}", org=(x, y), fontFace=cv2.FONT_HERSHEY_SIMPLEX , fontScale=1, color=(0, 0, 255), thickness=1, lineType=1)
    cv2.imshow('video', img)
    k = cv2.waitKey(30)

    if iter % 100 == 0 and sampling: # 100 is arbitrary. The point is to not sample to frequently.
        sample(faces, iter)

    if k == 27: # press 'ESC' to quit
        break
    iter += 1

cap.release()
cv2.destroyAllWindows()