### MAIN CV SCRIPT ###

# general imports
import sys
import random
import cv2
import threading
import numpy as np
import mediapipe as mp

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
    camera = cv2.VideoCapture("/dev/video0")
    camera.set(3, 1920)
    camera.set(4, 1080)
    return camera

def get_camera_input(camera):
    ret, frame = camera.read()
    return frame

def face_engagement_detection(face_model, landmark_predictor, face_2d, face_3d,  frame, engagement_record=[0]):
    # get faces
    faces = face_model.detectMultiScale(frame, scaleFactor=1.2, minNeighbors=5, minSize=(64, 64))
    # display boxes around faces
    # detect/display boxes around hands
    for (x_face, y_face, w_face, h_face) in faces:
        cv2.rectangle(frame, (x_face, y_face), (x_face+w_face, y_face+h_face), (255, 0, 0), 2)
        roi_color_face = frame[y_face:y_face+h_face, x_face:x_face+w_face]
       # cv2.putText(frame, f"Location: ({x_face}, {y_face})", org=(x_face, y_face-5), fontFace=cv2.FONT_HERSHEY_SIMPLEX , fontScale=1, color=(0, 0, 255), thickness=2, lineType=2)
        
        # Computing landmarks of a detected face and computing an engagement score
        landmark_results = landmark_predictor.process(frame)
        x, y, z = project_landmarks(landmark_results, frame, face_2d, face_3d)
        engagement = compute_engagement_score(x, y, z)
        engagement_record.append(engagement)
        cv2.putText(frame, f"Engagement:  {engagement:.3f}", org=(x_face, y_face-5), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 255), thickness=2, lineType=2)

    # Hand detection here, independent from the face detection.
   # t1 = threading.Thread(target=closed_hand_model, args=(closed_hand_model, frame, closed_hands))
   # t2 = threading.Thread(target=open_hand_model, args=(open_hand_model, frame, open_hands))

   # t1.start()
   # t2.start()
   # t1.join()
   # t2.join()
   # coords = hand_detector_mp(mp_model, frame, closed_hands)
   # closed_hands.append(coords)
    
   # if len(closed_hands) > 0:
   #     print(f"Hand Detected: {closed_hands[-1]}")
    
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


def format_data(raw_data):
    return

def send_to_pepper(hand_data, engagement):
    unique_hands = list(set(hand_data))
    
    for data in unique_hands:
        dic_data = {"bounding_box": (data[-1][0], data[-1][1], 100, 100),
                "frame_res": (1920, 1080),
                "confidence_score": -1}
        Info.Send("NumHands", {"value": 1})
        Info.Send("RaisedHandInfo", dic_data)
    return

def main():
    # init of CV module
   # PepperAPI.init("cv_node")
    print("API Initialised...")
    # set up camera
    camera = set_up_camera()
    
    # set up pre-trained models
    face_detect_model       = cv2.CascadeClassifier("./utils/models/face_detection.xml") # loading the pre-trained face detection model from OpenCV.
    closed_hand_raise_model = cv2.CascadeClassifier("./utils/models/closed_hand.xml")
    open_hand_raise_model   = cv2.CascadeClassifier("./utils/models/open_hand.xml")
    landmark_predictor = landmark_model()
    mp_hand_model = mp.solutions.hands.Hands()
    hands = []
    # main loop
    while True:
        frame  = get_camera_input(camera)
        face_3d = []
        face_2d = []

        coordinates = hand_detector_mp(mp_hand_model, frame)
        hands.append(coordinates)
        hands = list(filter(lambda x: x is not None, hands))

        frame2, engagement_list = face_engagement_detection(face_detect_model, landmark_predictor, face_2d, face_3d, frame)
        mean_engagement = np.mean(np.array(engagement_list))


        k = cv2.waitKey(30) 
        if k == 27: # press 'ESC' to quit
            break
        cv2.imshow("video", frame2)
        #cleaned_data = format_data(raw_data)
        
    print("Sending...") 
   # send_to_pepper(closed_hands)
    print("Sent")

    print("FINISHED")
   # print(list(set(closed_hands)))
   # print(list(set(open_hands)))

main()
