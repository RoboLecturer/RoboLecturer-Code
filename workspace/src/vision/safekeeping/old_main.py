from src.camera_sample import *
from utils import PepperAPI
from utils.PepperAPI import Info 
import sys
import random
#import rospy
  

cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
cap.set(3, 1920)
cap.set(4, 1080)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

face_detector = cv2.CascadeClassifier("utils/models/face_detection.xml") # loading the pre-trained face detection model from OpenCV.
print("face detector:" )
print(face_detector)
closed_hand_detector = cv2.CascadeClassifier("utils/models/closed_hand.xml")
open_hand_detector = cv2.CascadeClassifier("utils/models/open_hand.xml")
iter = 0

def simulate_cv_module(hand_raise_box, face_box=None, engagement_scores=None): 
	# Waits for trigger_hand_detection, sends hand_info.
    print("\nWaiting for trigger_hand_detection...") 
    if 1:#Info.Request("TriggerHandDetection"):
        if 1:#hand_raise_box != None:
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
            if 1:# Info.Request("TriggerAttentivenessDetection"):
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
            img = frame.astype("uint8")#np.random.rand(840, 840, 3)#cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # to make frames gray.
            print("img: ")
            print(img)
            try:
                faces = face_detector.detectMultiScale(img, scaleFactor=1.2, minNeighbors=5, minSize=(64, 64))
            except Exception as e:
                print(e)
                print("hi")
            engagement_scores = np.array([])

            print("before loop")
            for (x_face, y_face, w_face, h_face) in faces:
                print("after loop")
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
                print("CHECK1.5") 
                if open_hands[-1][1] < y_face:
                    cv2.putText(img, f"Question!", org=(open_hands[-1][0], open_hands[-1][1]-5), fontFace=cv2.FONT_HERSHEY_SIMPLEX , fontScale=1, color=(0, 255, 0), thickness=2, lineType=2)

                if closed_hands[-1][1] < y_face:
                    cv2.putText(img, f"Question!", org=(closed_hands[-1][0], closed_hands[-1][1]-5), fontFace=cv2.FONT_HERSHEY_SIMPLEX , fontScale=1, color=(0, 255, 0), thickness=2, lineType=2)
                
                engagement = random.uniform(0, 1) # Dummy variable, this will be a prediction in the future.
                np.append(engagement_scores, engagement)
                print("CHECK9")
                simulate_cv_module(hand_raise_box=closed_hands, face_box=(x_face, y_face, w_face, h_face), engagement_scores=engagement_scores)
            print("CHECK2")

            cv2.imshow('video', img)
            k = cv2.waitKey(0) 

            print("CHECK3")
            if iter % 100 == 0 and sampling: # 100 is arbitrary. The point is to not sample to frequently.
                sample(faces, iter)
            
            print("CHECK4")
            if k == 27: # press 'ESC' to quit  
                break
            iter += 1

        cap.release()
        cv2.destroyAllWindows()

    except:
        sys.exit(0)

