#! /usr/bin/env python3

#from src import camera_sample
from utils import PepperAPI
from utils.PepperAPI import Info 
import numpy as np
import cv2
import sys
import random
  

cap = cv2.VideoCapture(0)
cap.set(3, 1920)
cap.set(4, 1080)
face_detector = cv2.CascadeClassifier("utils/models/face_detection.xml") # loading the pre-trained face detection model from OpenCV.
closed_hand_detector = cv2.CascadeClassifier("utils/models/closed_hand.xml")
open_hand_detector = cv2.CascadeClassifier("utils/models/open_hand.xml")
iter = 0

def simulate_cv_module(hand_raise_box, face_box=None, engagement_scores=None): 
	# Waits for trigger_hand_detection, sends hand_info.
    print("\nWaiting for trigger_hand_detection...") 
    if Info.Request("TriggerHandDetection"):
        if hand_raise_box != None:
            print("\nSending hand info...")
            Info.Send("RaisedHandInfo", { 
                "bounding_box": hand_raise_box, 
                "frame_res": (1920, 1080), 
                "confidence_score": None # for now we don't provide a confidence of predicting a hand raise. 
            })
            return

        else: # no hand raised --> check the attention
            # trigger_noise_detection to Speech
            print("\nSend trigger_noise_detection...")
            Info.Send("TriggerNoiseDetection")

            # Waiting for a request from Pepper
            print("\nWaiting for trigger_attentiveness_detection...")
            if Info.Request("TriggerAttentivenessDetection"):
                print("\nSend FaceInfo...")
                Info.Send("FaceInfo", {
                "bounding_box": face_box,
                "frames_res": (1920, 1080),
                "engagement_score": engagement_scores[-1]
                })

                # Checking the overall engagement
                if engagement_scores.shape[0] >= 10: # at least 10 faces detected, this is part to be changed
                    mean_engagement = np.mean((engagement_scores.reverse())[:10]) # selecting 10 latest detection. Not ideal solution, this is only for demo.
                    if mean_engagement < 0.5: # below engagement threshold
                        random.sample([Info.Send("TriggerQuiz"), Info.Send("TriggerJoke")], 1) # randomly selecting an interaction with audience.

                    else: # engagement is above the desired threshold
                        Info.Send("IncrementLoopCounter") # Increasing the loop counter as an inattentiveness has not been detected.
            return

  

if __name__ == "__main__": 
	## Initialise CV node
    PepperAPI.init("cv_node")
    try:
        closed_hands = [(0, 0, 0, 0)]
        open_hands = [(0, 0 , 0 , 0)]
        while True:
            ret, frame = cap.read()
            img = frame#np.random.rand(840, 840, 3)#cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # to make frames gray.
            faces = face_detector.detectMultiScale(img, scaleFactor=1.2, minNeighbors=5, minSize=(64, 64))
            engagement_scores = np.array([])

            for (x_face, y_face, w_face, h_face) in faces:
                cv2.rectangle(img, (x_face, y_face), (x_face+w_face, y_face+h_face), (255, 0, 0), 2)
                roi_color_face = img[y_face:y_face+h_face, x_face:x_face+w_face] #input to the engagement measurer
                cv2.putText(img, f"Engagement: {0.69}", org=(x_face, y_face-5), fontFace=cv2.FONT_HERSHEY_SIMPLEX , fontScale=1, color=(0, 0, 255), thickness=2, lineType=2)
                
                # detecting two types of hand gestures in parallel
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
                
                engagement = random.uniform(0, 1) # Dummy variable, this will be a prediction in the future.
                np.append(engagement_scores, engagement)
                simulate_cv_module(hand_raise_box=closed_hands, face_box=(x_face, y_face, w_face, h_face), engagement_scores=engagement_scores)

            cv2.imshow('video', img)
            k = cv2.waitKey(30) 

            if iter % 100 == 0 and sampling: # 100 is arbitrary. The point is to not sample to frequently.
                sample(faces, iter)

            if k == 27: # press 'ESC' to quit  
                break
            iter += 1

            cap.release()
            cv2.destroyAllWindows()

    except:
        sys.exit(0)


# import numpy as np
# import cv2
# from sys import argv
# import rospy
# from sensor_msgs.msg import Image

# try:
#     sampling = argv[1]
# except Exception:
#     print("Sampling argument not provided")
#     sampling = False

# #cap = cv2.VideoCapture(0)
# #cap.set(3, 840)
# #cap.set(4, 840)
# face_detector = cv2.CascadeClassifier("face_detection.xml") # loading the pre-trained face detection model from OpenCV.
# iter = 0
# engagement = 0.69 # for now an arbitrary number. In the future, it will come from the classification model.

# def sample(faces, iter):
#     for (x, y, w, h) in faces:
#         cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
#         roi_color = img[y+8:y+h - 2, x+2:x+w-2] # these numbers were chosen so that the bounding boxes and text artifacts are removed from a face data.
#         cv2.imwrite(f"{x}_{y}_{iter}_face.jpg", roi_color)

# while True:
#     #ret, frame = cap.read()
#     #img = frame #cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # to make frames gray.
#     img = cv2.imread("per_normal_face.jpg")
#     faces = face_detector.detectMultiScale(img, scaleFactor=1.2, minNeighbors=5, minSize=(64, 64))
    
#     for (x, y, w, h) in faces:
#         cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
#         roi_color = img[y:y+h, x:x+w]
#         cv2.putText(img, f"Engagement: {engagement}", org=(x, y), fontFace=cv2.FONT_HERSHEY_SIMPLEX , fontScale=1, color=(0, 0, 255), thickness=1, lineType=1)
#     #cv2.imshow('video', img)
#     k = cv2.waitKey(30)

#     if iter % 100 == 0 and sampling: # 100 is arbitrary. The point is to not sample to frequently.
#         sample(faces, iter)

#     if k == 27: # press 'ESC' to quit
#         break
#     iter += 1
#     break

# cv2.imwrite("testy.jpg", img)

# #cap.release()
# cv2.destroyAllWindows()

# pub = rospy.Publisher("vision", Image, queue_size=10)
# rospy.init_node("node", anonymous=True)
# pub.publish(img)

# # import cv2
# # cap = cv2.VideoCapture(0)
# # print(cap.isOpened())

# # if not cap.isOpened():
# #     print("Camera cannot be opened")

# # while True:
# #     ret, frame = cap.read()
# #     if not ret:
# #         break
# #     cv2.imshow('frame', frame)

# #     if cv2.waitKey(1) == ord('q'):
# #         break

# # cap.release()
# # cv2.destroyAllWindows()


