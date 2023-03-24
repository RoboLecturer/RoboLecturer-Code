### MAIN CV SCRIPT ###

# general imports
import sys
import random
import cv2
import threading
import numpy as np
import mediapipe as mp
import argparse
import time


# PepperAPI imports
#import PepperAPI
#from PepperAPI import Info

# Importing our scripts
from src.head_pose_estimator import landmark_model, compute_engagement_score, project_landmarks

# global variables
closed_hands = []
open_hands   = []

# functions
def set_up_camera():
    camera = cv2.VideoCapture("/dev/video6")
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
    camera.set(3, 1080)
    camera.set(4, 720)
    return camera

def get_camera_input(camera):
    ret, frame = camera.read()
    return frame

def face_engagement_detection(face_model, landmark_predictor, face_2d, face_3d,  frame, engagement_record=[0]):
    # get faces
    faces = face_model.detectMultiScale(frame, scaleFactor=1.2, minNeighbors=5, minSize=(64, 64))

    # display boxes around faces
    for (x_face, y_face, w_face, h_face) in faces:
        cv2.rectangle(frame, (x_face, y_face), (x_face + w_face, y_face + h_face), (255, 0, 0), 2)
        roi_color_face = frame[y_face:y_face+h_face, x_face:x_face+w_face]
        
        # Computing landmarks of a detected face and computing an engagement score
        landmark_results = landmark_predictor.process(frame)
        x, y, z = project_landmarks(landmark_results, frame, face_2d, face_3d)
        engagement = compute_engagement_score(x, y, z)
        engagement_record.append(engagement)
        cv2.putText(frame, f"Engagement:  {engagement:.3f}", org=(x_face, y_face - 5), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 255), thickness=2, lineType=2)
    
    return frame, engagement_record


# Function for hand detection. If the model changes, make updates here. To use this, use threading.Thread method as well.
def hand_detector(cascade, frame, hands):
    detected_hands = cascade.detectMultiScale(frame, scaleFactor=1.2, minNeighbors=5, minSize=(64, 64))
    for (x, y, w, h) in detected_hands:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        hands.append((x, y, w, h))


def hand_detector_mp(model, frame): # Commented out the parts that killed the terminal.
    mp_drawing_utils = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    result = model.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    if result.multi_hand_landmarks:
        hand_landmark = result.multi_hand_landmarks[-1]
        for hand_landmark in result.multi_hand_landmarks:
            mp_drawing_utils.draw_landmarks(frame,
                                            hand_landmark,
                                            mp.solutions.hands.HAND_CONNECTIONS,
                                            mp_drawing_styles.get_default_hand_landmarks_style(),
                                            mp_drawing_styles.get_default_hand_connections_style())

        for idx, landmark in enumerate(hand_landmark.landmark):
             h, w, c = frame.shape
             cx, cy = int(landmark.x * w), int(landmark.y * h)

        return cx, cy

    return None


def send_to_pepper(hand_data):
    unique_hands = list(set(hand_data))
    
    for data in unique_hands:
        dic_data = {"bounding_box": (data[-1][0], data[-1][1], 100, 100),
                "frame_res": (1080, 720),
                "confidence_score": -1}
        Info.Send("NumHands", {"value": 1})
        Info.Send("RaisedHandInfo", dic_data)
    return

def main(camera, test_model, mp_hand_model, landmark_predictor):
   # closed_hand_raise_model = cv2.CascadeClassifier("./utils/models/closed_hand.xml")
   # open_hand_raise_model   = cv2.CascadeClassifier("./utils/models/open_hand.xml")
    hands = []
    # main loop
    while True:
        frame  = get_camera_input(camera)
        face_3d = []
        face_2d = []

        coordinates = hand_detector_mp(mp_hand_model, frame)
        print(f"Detected hand: {coordinates}")
        hands.append(coordinates)
        hands = list(filter(lambda x: x is not None, hands))

        frame2, engagement_list = face_engagement_detection(test_model, landmark_predictor, face_2d, face_3d, frame)
        mean_engagement = np.mean(np.array(engagement_list))


        k = cv2.waitKey(30) 
        if k == 27: # press 'ESC' to quit
            break
        cv2.imshow("video", frame2)
        
    print("Sending...") 
   # send_to_pepper(closed_hands)
    print("Sent")

    print("FINISHED")

if __name__ == "__main__":
   # PepperAPI.init("cv_node")
    print("API Initialised...")
    # Setting up pre-trained models and camera.
    camera = set_up_camera()
    face_model = cv2.CascadeClassifier("./utils/models/face_detection.xml")
    hand_model = mp.solutions.hands.Hands(model_complexity=0, max_num_hands=6, min_detection_confidence=0.1)
    landmark_predictor = landmark_model()
    # Running the main detection script.
    main(camera, face_model, hand_model, landmark_predictor)
