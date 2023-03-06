### MAIN CV SCRIPT ###

# general imports
import sys
import random
import cv2
import threading
import numpy as np
import mediapipe as mp
import time

# PepperAPI imports
import PepperAPI
from PepperAPI import Info

# Importing our scripts
from src.head_pose_estimator import landmark_model, compute_engagement_score, project_landmarks

# global variables
closed_hands = []
open_hands   = []

# functions
def set_up_camera():
    camera = cv2.VideoCapture("/dev/video0")
    camera.set(3, 1920)
    camera.set(4, 1080)
    return camera

def get_camera_input(camera):
    ret, frame = camera.read()
    return frame

def face_engagement_detection(face_model, landmark_predictor, face_2d, face_3d,  frame, engagement_record=[]):
    # get faces
    faces = face_model.detectMultiScale(frame, scaleFactor=1.2, minNeighbors=5, minSize=(64, 64))
    # display boxes around faces
    for (x_face, y_face, w_face, h_face) in faces:
        cv2.rectangle(frame, (x_face, y_face), (x_face+w_face, y_face+h_face), (255, 0, 0), 2)
        roi_color_face = frame[y_face:y_face+h_face, x_face:x_face+w_face]
        
        # Computing landmarks of a detected face and computing an engagement score
        landmark_results = landmark_predictor.process(frame)
        x, y, z = project_landmarks(landmark_results, frame, face_2d, face_3d)
        engagement = compute_engagement_score(x, y, z)
        engagement_record.append(engagement)
        cv2.putText(frame, f"Engagement:  {engagement:.3f}", org=(x_face, y_face-5), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 255), thickness=2, lineType=2)
    
    return frame, engagement_record

# Function for hand detection. If the model changes, make updates here.
def hand_detector(cascade, frame, hands):
    detected_hands = cascade.detectMultiScale(frame, scaleFactor=1.2, minNeighbors=5, minSize=(64, 64))
    for (x, y, w, h) in detected_hands:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        hands.append((x, y, w, h))


def hand_detector_mp(model, frame):
   # mp_drawing_utils = mp.solutions.drawing_utils
   # mp_drawing_styles = mp.solutions.drawing_styles
    result = model.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    if result.multi_hand_landmarks:
        hand_landmark = result.multi_hand_landmarks[-1]
       # for hand_landmark in result.multi_hand_landmarks:
           # mp_drawing_utils.draw_landmarks(frame,
           #                                 hand_landmark,
           #                                 model.HAND_CONNECTIONS,
           #                                 mp_drawing_styles.get_default_hand_landmarks_style(),
           #                                 mp_drawing_styles.get_default_hand_connections_style())

        for idx, landmark in enumerate(hand_landmark.landmark):
             h, w, c = frame.shape
             cx, cy = int(landmark.x * w), int(landmark.y * h)

        return cx, cy

    return None


def send_to_pepper(hand_data):
    unique_hands = list(set(hand_data))    
    for data in unique_hands:
        dic_data = {"bounding_box": (data[0], data[1], 100, 100),
                "frame_res": (1920, 1080),
                "confidence_score": -1}
        #Info.Send("NumHands", {"value": 1}) # Now it's one, because mediapipe hand detector only does that for one.
        Info.Send("RaisedHandInfo", dic_data)
    return


def main(camera, face_detect_model, mp_hand_model, landmark_predictor):
    hands = []
    # main loop
    while True:
        #frame  = get_camera_input(camera)
        face_3d = []
        face_2d = []

        Info.Request("State", {"name": "Start"})
        Info.Request("TriggerHandDetection")

        start = time.time()
        while True:
            frame = get_camera_input(camera)
            coordinates = hand_detector_mp(mp_hand_model, frame)
            end = time.time()
            if end - start > 10:
                break
        hands.append(coordinates)
        hands = list(filter(lambda x: x is not None, hands))
        
        num_hands = len(hands)
        if num_hands == 0:
            Info.Send("State", {"AnyQuestions": "NoHandsRaised"})

        else:
            Info.Send("State", {"AnyQuestions": "HandsRaised"})
            Info.Send("NumHands", {"value": num_hands})
            send_to_pepper(hands)

            while Info.Request("State", {"name": "AnyQuestions", "print": False}) != "NoHandsRaised":
                pass

        state = Info.Request("State", {"name": "NoiseLevel"})

        if state == "High":
            return

        # Start checking the engagement/attentiveness of the class
        frame2, engagement_list = face_engagement_detection(face_detect_model, landmark_predictor, face_2d, face_3d, frame)
        mean_engagement = np.mean(np.array(engagement_list))

        if mean_engagement < 0.5: # if the mean engagement is below the threshold, send the non-attention alert.
            Info.Send("State", {"Attentiveness": "NotAttentive"})
            return

        else:
            Info.Send("State", {"Attentiveness": "Attentive"})

        Info.Request("State", {"name": "NoQuestionsLoop"})
        return
    
       # k = cv2.waitKey(30)
       # cv2.imshow("video", frame2)

       # if k == 27:
       #     break

       # Info.Request("State", {"name": "NoQuestionsLoop"})
       # return



if __name__ == "__main__":
    # Initialising the connection.
    PepperAPI.init("cv_node")
    print("API Initialised")
    # Setting up the camera and pretrained models.
    camera = set_up_camera()
    face_model = cv2.CascadeClassifier("./utils/models/face_detection.xml")
    hand_model = mp.solutions.hands.Hands()
    landmark_predictor = landmark_model()
    main(camera, face_model, hand_model, landmark_predictor)
