### MAIN CV SCRIPT ###

# general imports
import sys
import random
import cv2
import threading

# PepperAPI imports
from utils import PepperAPI
from utils.PepperAPI import Info 

# global variables
closed_hands = [(0, 0)]
open_hands   = [(0, 0)]

# functions
def set_up_camera():
    camera = cv2.VideoCapture(0)
    camera.set(3, 1920)
    camera.set(4, 1080)
    return camera

def get_camera_input(camera):
    ret, frame = camera.read()
    return frame

def hand_raise_detection(face_model, closed_hand_model, opened_hand_model, frame):
    # get faces
    faces = face_model.detectMultiScale(frame, scaleFactor=1.2, minNeighbors=5, minSize=(64, 64))
    
    # display boxes around faces
    # detect/display boxes around hands
    for (x_face, y_face, w_face, h_face) in faces:
        cv2.rectangle(frame, (x_face, y_face), (x_face+w_face, y_face+h_face), (255, 0, 0), 2)
        roi_color_face = frame[y_face:y_face+h_face, x_face:x_face+w_face]
        cv2.putText(frame, f"Location: ({x_face}, {y_face})", org=(x_face, y_face-5), fontFace=cv2.FONT_HERSHEY_SIMPLEX , fontScale=1, color=(0, 0, 255), thickness=2, lineType=2)

        t1 = threading.Thread(target=hand_detector, args=(closed_hand_model, frame, closed_hands))
        t2 = threading.Thread(target=hand_detector, args=(opened_hand_model, frame, open_hands))
        
        t1.start()
        t2.start()
        t1.join()
        t2.join()
        
        print(f"Face detected: {(x_face, y_face)}")
        print(f"Hand detected: {(closed_hands[-1])}")
    
    return frame

def hand_detector(cascade, frame, hands):
    detected_hands = cascade.detectMultiScale(frame, scaleFactor=1.2, minNeighbors=5, minSize=(64, 64))
    for (x, y, w, h) in detected_hands:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        hands.append((x, y))

def format_data(raw_data):
    return

def send_to_pepper(data):
    Info.send(data)
    return

def main():
    # init of CV module
    PepperAPI.init("cv_node")
    
    # set up camera
    camera = set_up_camera()
    
    # set up pre-trained models
    face_detect_model       = cv2.CascadeClassifier("./utils/models/face_detection.xml") # loading the pre-trained face detection model from OpenCV.
    closed_hand_raise_model = cv2.CascadeClassifier("./utils/models/closed_hand.xml")
    open_hand_raise_model   = cv2.CascadeClassifier("./utils/models/open_hand.xml")
    
    # main loop
    while 1:
        frame  = get_camera_input(camera)
        frame2 = hand_raise_detection(face_detect_model, open_hand_raise_model, closed_hand_raise_model, frame)
        
        cv2.imshow('video', frame2)
        
        k = cv2.waitKey(30) 
        if k == 27: # press 'ESC' to quit
            break
        
        #cleaned_data = format_data(raw_data)
        
    
    send_to_pepper(closed_hands)

    print("FINISHED")
    print(closed_hands)
    print(open_hands)

main()