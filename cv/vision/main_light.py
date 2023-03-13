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
    camera = cv2.VideoCapture("/dev/video6")
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("H", "J", "P", "G"))
    camera.set(3, 1080)
    camera.set(4, 720)
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




### YOLO main

""" 
### Run --> python main.py --weights utils/models/YOLOface/weights/yolov7s-face.pt --save-txt --nosave
### Print CPU/GPU device: utils/models/YOLOface/utils/torch_utils.py --> select_device() --> uncomment logger.info (line 85)
### Print total number of faces + inference time per frame: utils/models/YOLOface/detect.py --> uncomment print command (line 162)

import argparse
import numpy as np
import cv2

from utils.models.YOLOface.detect import detect, config_net
from utils.models.YOLOface.utils.general import check_requirements
from utils.models.YOLOface.utils.general import check_requirements

parser = argparse.ArgumentParser()
parser.add_argument('--weights', nargs='+', type=str, default='yolov5s.pt', help='model.pt path(s)')
parser.add_argument('--source', type=str, default='utils/models/YOLOface/data/images', help='source')  # file/folder, 0 for webcam
parser.add_argument('--img-size', nargs= '+', type=int, default=640, help='inference size (pixels)')
parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')                               
parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')                                  
parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')                               
parser.add_argument('--view-img', action='store_true', help='display results')                                                      
parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')                                        
parser.add_argument('--save-txt-tidl', action='store_true', help='save results to *.txt in tidl format')                                
parser.add_argument('--save-bin', action='store_true', help='save base n/w outputs in raw bin format')                              
parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')                       
parser.add_argument('--save-crop', action='store_true', help='save cropped prediction boxes')                               
parser.add_argument('--nosave', action='store_true', help='do not save images/videos')                                          
parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')                              
parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')                                   
parser.add_argument('--augment', action='store_true', help='augmented inference')                                                   
parser.add_argument('--update', action='store_true', help='update all models')                                                      
parser.add_argument('--project', default='runs/detect', help='save results to project/name')                                            
parser.add_argument('--name', default='exp', help='save results to project/name')                                               
parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')                                   
parser.add_argument('--line-thickness', default=1, type=int, help='bounding box thickness (pixels)')                                        
parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')                                            
parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')                             
parser.add_argument('--kpt-label', type=int, default=5, help='number of keypoints')                                             
opt = parser.parse_args()                                                                                                       
print(opt)
check_requirements(exclude=('tensorboard', 'pycocotools', 'thop'))

model, imgsz, stride = config_net(opt=opt)

# Defining frame sizes
frame_size_x = 640
frame_size_y = 640

# Replace for the camera loop
for i in range(10):
    frame = cv2.imread('utils\models\YOLOface\data\images\classroom_01.jpg')

    # Returns the list of all faces coordinates - normalized values
    faces_coords = detect(opt, frame, model, imgsz, stride)
    faces_coords = np.array(faces_coords)

# Getting the coordinates in pixels
faces_coords[:,0] *= frame_size_x     # Center x of each bounding box
faces_coords[:,1] *= frame_size_y     # Center y of each bounding box
faces_coords[:,2] *= frame_size_x     # Height of each bounding box
faces_coords[:,3] *= frame_size_y     # Width  of each bounding box
faces_coords = faces_coords.astype(int)

#print(faces_coords)
#print(10*faces_coords[:,1])
 """
