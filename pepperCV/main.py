from src.camera_sample import *
from utils import PepperAPI
from utils.PepperAPI import Info 
import time, sys 
#import rospy
  

cap = cv2.VideoCapture(0)
cap.set(3, 1920)
cap.set(4, 1080)
face_detector = cv2.CascadeClassifier("utils/models/face_detection.xml") # loading the pre-trained face detection model from OpenCV.
closed_hand_detector = cv2.CascadeClassifier("utils/models/closed_hand.xml")
open_hand_detector = cv2.CascadeClassifier("utils/models/open_hand.xml")
iter = 0
engagement = 0.69

def simulate_cv_module(hand_raise_box, face_box=None, engagement=None): 
	# Waits for trigger_hand_detection, sends hand_info.
    print("\nWaiting for trigger_hand_detection...") 
    if Info.Request("TriggerHandDetection"):
        time.sleep(2)
        print("\nSending hand info...")
        Info.Send("RaisedHandInfo", { 
		    "bounding_box": hand_raise_box, 
		    "frame_res": (1920, 1080), 
		    "confidence_score": None # for now we don't provide a confidence of predicting a hand raise. 
	    })
        time.sleep(2)
        Info.Send("IncrementLoopCounter") # Increasing the loop counter as an inattentiveness has not been detected.
        return

    else:
        print("\nWaiting for trigger_attentiveness_detection...")
        if not Info.Request("TriggerAttentivenessDetection"):
            return

        # trigger_noise_detection to Speech
        time.sleep(2)
        print("\nSend trigger_noise_detection...")
        Info.Send("TriggerNoiseDetection")

        time.sleep(2)
        print("\nSend FaceInfo...")
        Info.Send("FaceInfo", {
            "bounding_box": face_box,
            "frames_res": (1920, 1080),
            "engagement_score": engagement
        })
        return 
  

if __name__ == "__main__": 
	## Initialise CV node
    PepperAPI.init("cv_node")
    try:
        while True:
            ret, frame = cap.read()
            img = frame#np.random.rand(840, 840, 3)#cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # to make frames gray.
            faces = face_detector.detectMultiScale(img, scaleFactor=1.2, minNeighbors=5, minSize=(64, 64))

            for (x_face, y_face, w_face, h_face) in faces:
                cv2.rectangle(img, (x_face, y_face), (x_face+w_face, y_face+h_face), (255, 0, 0), 2)
                roi_color_face = img[y_face:y_face+h_face, x_face:x_face+w_face]
                cv2.putText(img, f"Engagement: {0.69}", org=(x_face, y_face-5), fontFace=cv2.FONT_HERSHEY_SIMPLEX , fontScale=1, color=(0, 0, 255), thickness=2, lineType=2)
                
                # detecting two types of hand gestures in parallel
                closed_hands = [(0, 0)]
                open_hands = [(0, 0)]
                t1 = threading.Thread(target=hand_detector, args=(closed_hand_detector, img, closed_hands))
                t2 = threading.Thread(target=hand_detector, args=(open_hand_detector, img, open_hands))
                t1.start()
                t2.start()
                t1.join()
                t2.join()

                if open_hands[-1][1] < y_face:
                    cv2.putText(img, f"Question!", org=(open_hands[-1][0], open_hands[-1][1]-5), fontFace=cv2.FONT_HERSHEY_SIMPLEX , fontScale=1, color=(0, 255, 0), thickness=2, lineType=2)
                    simulate_cv_module(open_hands[-1], face_box=(x_face, y_face, w_face, h_face), engagement=0.69) # there might be problem in swapping of detected hands.

                if closed_hands[-1][1] < y_face:
                    cv2.putText(img, f"Question!", org=(closed_hands[-1][0], closed_hands[-1][1]-5), fontFace=cv2.FONT_HERSHEY_SIMPLEX , fontScale=1, color=(0, 255, 0), thickness=2, lineType=2)
                    simulate_cv_module(closed_hands[-1], face_box=(x_face, y_face, w_face, h_face), engagement=0.69) # there might be problem in swapping of detected hands.
                
                simulate_cv_module(hand_raise_box=None, face_box=(x_face, y_face, w_face, h_face), engagement=0.69) # calling a function when no hand raise was detected.

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
