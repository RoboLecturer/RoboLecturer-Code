import numpy as np
import cv2
from sys import argv
import os
import threading


try:
    sampling = argv[1]
except Exception:
    print("Sampling argument not provided")
    sampling = False

cap = cv2.VideoCapture(0)
cap.set(3, 1920)
cap.set(4, 1080)

face_detector = cv2.CascadeClassifier("../utils/models/face_detection.xml") # loading the pre-trained face detection model from OpenCV.
closed_hand_detector = cv2.CascadeClassifier("../utils/models/closed_hand.xml")
open_hand_detector = cv2.CascadeClassifier("../utils/models/open_hand.xml")
iter = 0
engagement = 0.69 # for now an arbitrary number. In the future, it will come from the classification model.

def sample(faces, iter):
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
        roi_color = img[y+8:y+h - 2, x+2:x+w-2] # these numbers were chosen so that the bounding boxes and text artifacts are removed from a face data.
        cv2.imwrite(f"{x}_{y}_{iter}_face.jpg", roi_color)


def hand_detector(cascade, frame, hands):
    detected_hands = cascade.detectMultiScale(frame, scaleFactor=1.2, minNeighbors=5, minSize=(64, 64))
    for (x, y, w, h) in detected_hands:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        hands.append((x, y))


while True:
    ret, frame = cap.read()
    img = frame#np.random.rand(840, 840, 3)#cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # to make frames gray.
    faces = face_detector.detectMultiScale(img, scaleFactor=1.2, minNeighbors=5, minSize=(64, 64))
    
    
    for (x_face, y_face, w_face, h_face) in faces:
        cv2.rectangle(img, (x_face, y_face), (x_face+w_face, y_face+h_face), (255, 0, 0), 2)
        roi_color_face = img[y_face:y_face+h_face, x_face:x_face+w_face]
        cv2.putText(img, f"Confidence: {0.69}", org=(x_face, y_face-5), fontFace=cv2.FONT_HERSHEY_SIMPLEX , fontScale=1, color=(0, 0, 255), thickness=2, lineType=2)

        closed_hands = [(0, 0)]
        open_hands = [(0, 0)]
        t1 = threading.Thread(target=hand_detector, args=(closed_hand_detector, img, closed_hands))
        t2 = threading.Thread(target=hand_detector, args=(open_hand_detector, img, open_hands))
        t1.start()
        t2.start()

        t1.join()
        t2.join()

        if open_hands[-1][1] < y_face:
            cv2.putText(img, f"Question!", org=(open_hands[-1][0], open_hands[-1][1]-5), fontFace=cv2.FONT_HERSHEY_SIMPLEX , fontScale=1, color=(0, 255, 0), thickness=2, lineType=2)

        if closed_hands[-1][1] < y_face:
            cv2.putText(img, f"Question!", org=(closed_hands[-1][0], closed_hands[-1][1]-5), fontFace=cv2.FONT_HERSHEY_SIMPLEX , fontScale=1, color=(0, 255, 0), thickness=2, lineType=2)



    cv2.imshow('video', img)
    k = cv2.waitKey(30) 

    if iter % 100 == 0 and sampling: # 100 is arbitrary. The point is to not sample to frequently.
        sample(faces, iter)

    if k == 27: # press 'ESC' to quit  
        break
    iter += 1

cap.release()
cv2.destroyAllWindows()