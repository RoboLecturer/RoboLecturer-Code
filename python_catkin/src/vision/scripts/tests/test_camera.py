import numpy as np
import cv2
from sys import argv
import os
import threading
import random
import time
import pandas as pd
from functools import wraps

execution_time=[]

def time_inference(func):
    @wraps(func)
    def timeit_wrapper(*args, **kwargs):
        start__inference = time.perf_counter()
        result = func(*args, **kwargs)
        end_inference = time.perf_counter()
        total_time_inference = end_inference - start__inference
        total_time_inference = total_time_inference * 1000
        execution_time.append(total_time_inference)
        print(f'Function {func.__name__} took {total_time_inference:.4f} ms')
        return result
    return timeit_wrapper

try:
    sampling = argv[1]
except Exception:
    print("Sampling argument not provided")
    sampling = False

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
cap.set(3, 1920)
cap.set(4, 1080)

face_detector = cv2.CascadeClassifier("../utils/models/face_detection.xml") # loading the pre-trained face detection model from OpenCV.
closed_hand_detector = cv2.CascadeClassifier("../utils/models/closed_hand.xml")
open_hand_detector = cv2.CascadeClassifier("../utils/models/open_hand.xml")
iter = 0

def sample(faces, iter):
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
        roi_color = img[y+8:y+h - 2, x+2:x+w-2] # these numbers were chosen so that the bounding boxes and text artifacts are removed from a face data.
        cv2.imwrite(f"{x}_{y}_{iter}_face.jpg", roi_color)


#@time_inference
def detect_face(img, model):
    return model.detectMultiScale(img, scaleFactor=1.2, minNeighbors=5, minSize=(64, 64))

#@time_inference
def hand_detector(cascade, frame, hands):
    detected_hands = cascade.detectMultiScale(frame, scaleFactor=1.2, minNeighbors=5, minSize=(64, 64))
    for (x, y, w, h) in detected_hands:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        hands.append((x, y))

@time_inference
def read_video(cap):
    return cap.read()

closed_hands = [(0, 0, 0, 0)]
open_hands = [(0, 0, 0, 0)]

while True:
    ret, frame = read_video(cap)
    img = frame#np.random.rand(840, 840, 3)#cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # to make frames gray.
    faces = detect_face(img, face_detector)
    engagement = round(random.uniform(0, 1), 3)
    
    for (x_face, y_face, w_face, h_face) in faces:
        cv2.rectangle(img, (x_face, y_face), (x_face+w_face, y_face+h_face), (255, 0, 0), 2)
        roi_color_face = img[y_face:y_face+h_face, x_face:x_face+w_face]
        cv2.putText(img, f"Engagement: {engagement}", org=(x_face, y_face-5), fontFace=cv2.FONT_HERSHEY_SIMPLEX , fontScale=1, color=(0, 0, 255), thickness=2, lineType=2)

        t1 = threading.Thread(target=hand_detector, args=(closed_hand_detector, img, closed_hands))
        t2 = threading.Thread(target=hand_detector, args=(open_hand_detector, img, open_hands))
        t1.start()
        t2.start()

        t1.join()
        t2.join()
        #print(f"Face detected: {(x_face, y_face)}")
        #print(f"Hand detected: {(closed_hands[-1])}")

        if open_hands[-1][1] < y_face:
            cv2.putText(img, f"Question!", org=(open_hands[-1][0], open_hands[-1][1]-5), fontFace=cv2.FONT_HERSHEY_SIMPLEX , fontScale=1, color=(0, 255, 0), thickness=2, lineType=2)

        if closed_hands[-1][1] < y_face:
            cv2.putText(img, f"Question!", org=(closed_hands[-1][0], closed_hands[-1][1]-5), fontFace=cv2.FONT_HERSHEY_SIMPLEX , fontScale=1, color=(0, 255, 0), thickness=2, lineType=2)



    cv2.imshow('video', img)
    k = cv2.waitKey(30) 

    if iter % 100 == 0 and sampling: # 100 is arbitrary. The point is to not sample to frequently.
        sample(faces, iter)

    if k == 27: # press 'ESC' to quit
        pd.DataFrame(np.array(execution_time)).to_csv("execution.csv", index=False)
        break
    iter += 1

cap.release()
cv2.destroyAllWindows()