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

def sample(faces, iter):
    for (x, y, w, h) in faces:
        cv2.rectangle(faces, (x, y), (x+w, y+h), (255, 0, 0), 2)
        roi_color = faces[y+8:y+h - 2, x+2:x+w-2] # these numbers were chosen so that the bounding boxes and text artifacts are removed from a face data.
        cv2.imwrite(f"{x}_{y}_{iter}_face.jpg", roi_color)


def hand_detector(cascade, frame, hands):
    detected_hands = cascade.detectMultiScale(frame, scaleFactor=1.2, minNeighbors=5, minSize=(64, 64))
    for (x, y, w, h) in detected_hands:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        hands.append((x, y))

