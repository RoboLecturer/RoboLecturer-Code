### MAIN CV SCRIPT ###

# general imports
import cv2
import numpy as np
import mediapipe as mp
import time

# PepperAPI imports
import PepperAPI
from PepperAPI import Info
import argparse
# Importing our scripts
from src.head_pose_estimator import engagement_from_landmarks
from src.reject_points import reject_points
from utils.models.YOLOface.detect import detect, config_net
from utils.models.YOLOface.utils.general import check_requirements

parser = argparse.ArgumentParser()
parser.add_argument("--weights", nargs="+", type=str, default="yolov5s.pt", help="model.pt path(s)")
parser.add_argument("--source", type=str, default="utils/models/YOLOface/data/images", help="source")
parser.add_argument("--img-size", nargs="+", type=int, default=640, help="inference size (pixels)")


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
check_requirements(exclude=("tensorboard", "pycocotools", "thop"))

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


def face_engagement_detection(face_model, frame, opt, imgsz, stride, engagement_record=[0]):
    # get faces
    faces = detect(opt, frame, face_model, imgsz, stride) # YoloV7
    faces = np.array(faces)

    # display boxes around faces
    if len(faces) != 0: # if a face was detected.
        for (x1, y1, x2, y2) in faces[:, 0:4]:
            x_centre = (x1 + x2) / 2
            y_centre = (y1 + y2) / 2
            #cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
            # Computing landmarks of a detected face and computing an engagement score
            landmark_results_detected = faces[:, 4:]
            engagement = engagement_from_landmarks(landmark_results_detected, frame, x_centre, y_centre)
            engagement_record.append(engagement)
        
    return frame, engagement_record


def hand_detector(model, frame): # Commented out the parts that killed the terminal.
    knuckle_idx = [1, 5, 9, 13, 17]
    fingertip_idx = [4, 8, 12, 16, 20]
    dist_fingers = np.array([])
    result = model.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    if result.multi_hand_landmarks:
        n_hands = np.array(result.multi_hand_landmarks)
        hand_landmarks = np.array([0, 0])
        for i in range(n_hands.shape[0]):
            hand_landmark = n_hands[i]

            for idx, landmark in enumerate(hand_landmark.landmark):
                h, w, c = frame.shape
                if idx == 0:
                    x_base = int(landmark.x * w)
                    y_base = int(landmark.y * h)

                elif idx in knuckle_idx:
                    x_knuckle = int(landmark.x * w)
                    y_knuckle = int(landmark.y * h)

                elif idx in fingertip_idx:
                    x_tip = int(landmark.x * w)
                    y_tip = int(landmark.y * h)

                    dist_to_knuckle = np.linalg.norm(np.array([x_tip, y_tip]) - np.array([x_knuckle, y_knuckle]))
                    dist_to_base = np.linalg.norm(np.array([x_tip, y_tip]) - np.array([x_base, y_base]))
                    dist_proportion = dist_to_knuckle / dist_to_base
                    dist_fingers = np.append(dist_fingers, dist_proportion)

            metric = np.mean(dist_fingers)
            #print(f"Metric: {metric}")
            if metric < 0.535:
                hand_landmarks = np.vstack([hand_landmarks, np.array([x_base, y_base])])
        
        return hand_landmarks[1:]

    return np.array([0, 0])


def send_to_pepper(hand_data):
    for data in hand_data:
        dic_data = {"bounding_box": (data[0], data[1], 100, 100),
                "frame_res": (1080, 720),
                "confidence_score": -1}
        Info.Send("RaisedHandInfo", dic_data)
    return


def main(camera, face_detect_model, mp_hand_model, landmark_predictor, opt, imgsz, stride):
    hands = np.array([0, 0])
    # main loop
    while True:
        Info.Request("State", {"name": "Start"})
        Info.Request("TriggerHandDetection")

        start = time.time()
        while True:
            frame = get_camera_input(camera)
            coordinates = hand_detector(mp_hand_model, frame)
            end = time.time()
            if end - start > 10:
                break

        hands = np.vstack([hands, coordinates])
        hands = reject_points(hands, threshold=20)
        hands = np.unique(hands, axis=0)
        
        num_hands = len(hands)
        if num_hands == 0:
            Info.Send("State", {"AnyQuestions": "NoHandsRaised"})

        else:
            Info.Send("State", {"AnyQuestions": "HandsRaised"})
            Info.Send("NumHands", {"value": num_hands})
            send_to_pepper(hands)
            hands = np.array([0, 0])

            while Info.Request("State", {"name": "AnyQuestions", "print": False}) != "NoHandsRaised":
                pass

        state = Info.Request("State", {"name": "NoiseLevel"})

        if state == "High":
            continue

        # Start checking the engagement/attentiveness of the class
        frame2, engagement_list = face_engagement_detection(face_detect_model, frame, opt, imgsz, stride)
        mean_engagement = np.mean(np.array(engagement_list))

        if mean_engagement < 0.5: # if the mean engagement is below the threshold, send the non-attention alert.
            Info.Send("State", {"Attentiveness": "NotAttentive"})
            continue

        else:
            Info.Send("State", {"Attentiveness": "Attentive"})

        Info.Request("State", {"name": "NoQuestionsLoop"})
        continue



if __name__ == "__main__":
    # Initialising the connection.
    PepperAPI.init("cv_node")
    print("API Initialised")
    # Setting up the camera and pretrained models.
    camera = set_up_camera()
    yolo_face_model, imgsz, stride = config_net(opt=opt)
    hand_model = mp.solutions.hands.Hands(model_complexity=0, max_num_hands=15, min_detection_confidence=0.1)
    main(camera, yolo_face_model, hand_model, opt, imgsz, stride)


