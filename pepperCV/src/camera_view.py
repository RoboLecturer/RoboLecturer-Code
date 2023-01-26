import numpy as np
import cv2

cap = cv2.VideoCapture(0)
cap.set(3, 340)
cap.set(4, 480)
face_detector = cv2.CascadeClassifier("face_detection.xml")

while True:
    ret, frame = cap.read()
    img = frame#cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_detector.detectMultiScale(img, scaleFactor=1.2, minNeighbors=5, minSize=(32, 32))
    
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
        roi_color = img[y:y+h, x:x+w]
    cv2.imshow('video', img)
    k = cv2.waitKey(30)
    if k == 27: # press 'ESC' to quit
        cv2.imwrite("face.jpg", roi_color)
        break
cap.release()
cv2.destroyAllWindows()