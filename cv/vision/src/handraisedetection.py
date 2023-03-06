import cv2
import mediapipe as mp
import time 

def hand_raise_detection():
    mp_hand = mp.solutions.hands
    hands = mp_hand.Hands()

    mp_drawing_utils = mp.solutions.drawing_utils

    mp_drawing_styles = mp.solutions.drawing_styles

    cap = cv2.VideoCapture(0)

    prev_time = 0

    while True:

        success, img = cap.read()

        if not success:
            break

        result = hands.process(cv2.cvtColor(img,cv2.COLOR_BGR2RGB))

        #print(result.multi_hand_landmarks)

        if result.multi_hand_landmarks:
            for hand_landmark in result.multi_hand_landmarks:
                mp_drawing_utils.draw_landmarks(img, 
                                                hand_landmark, 
                                                mp_hand.HAND_CONNECTIONS,
                                                mp_drawing_styles.
                                                get_default_hand_landmarks_style(),
                                                mp_drawing_styles.
                                                get_default_hand_connections_style()
                                                )


            for id, landmark in enumerate(hand_landmark.landmark):
                #print(id, landmark)
                h, w, c = img.shape
                cx, cy = int(landmark.x*w), int(landmark.y*h)
                #print(cx, cy)

        #Only if you want to know the FPS score
        #cur_time = time.time()
        #FPS = 1/(cur_time - prev_time)
        #prev_time = cur_time
        #cv2.putText(img, f"FPS: {str(FPS)}", (100,100), cv2.FONT_HERSHEY_COMPLEX, 2, (200, 200), 3)
        
        k = cv2.waitKey(30)
        if k == 27: # press 'ESC' to quit
            break

        cv2.imshow("Image",img)

    cap.release()
    cv2.destroyAllWindows()
    
    #coordinates of hand within the camera frame
    return cx, cy

if __name__ == "__main__":
    coord = hand_raise_detection()
    print("Coordinates: ", coord)
