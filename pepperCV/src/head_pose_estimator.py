import mediapipe as mp
import cv2
import numpy as np
import time

mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5)

mp_drawing = mp.solutions.drawing_utils

drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)

cap = cv2.VideoCapture(0)
cap.set(3, 1920)
cap.set(4, 1080)


def compute_engagement_score(x, y, z):
    return 1.0/(np.cosh(0.1*x)) * 1.0/(np.cosh(0.1*y)) * 1.0/(np.cosh(0.1*z))

while cap.isOpened():
    ret, img = cap.read()

    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    img.flags.writeable = False

    results = face_mesh.process(img)

    img.flags.writeable = True

    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    img_h, img_w, img_c = img.shape
    face_3d = []
    face_2d = []

    if results.multi_face_landmarks:
        for face_landmarks in results.multi_face_landmarks:
            for idx, lm in enumerate(face_landmarks.landmark):
                if idx == 33 or idx == 263 or idx == 1 or idx == 61 or idx == 291 or idx == 199:
                    if idx == 1:
                        nose_2d = (lm.x * img_w, lm.y * img_h)
                        nose_3d = (lm.x * img_w, lm.y * img_h, lm.z * 3000)

                    x, y = int(lm.x * img_w), int(lm.y * img_h)

                    face_2d.append([x, y])

                    face_3d.append([x, y, lm.z])
            
            face_2d = np.array(face_2d, dtype=np.float64)

            face_3d = np.array(face_3d, dtype=np.float64)

            focal_length = 1 * img_w

            cam_matrix = np.array([[focal_length, 0, img_h / 2],
                                    [0, focal_length, img_w / 2],
                                    [0, 0, 1]])

            dist_matrix = np.zeros((4, 1), dtype=np.float64)

            ret, rot_vec, trans_vec = cv2.solvePnP(face_3d, face_2d, cam_matrix, dist_matrix)

            rmat, jac = cv2.Rodrigues(rot_vec)

            angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(rmat)

            x = angles[0] * 360
            y = angles[1] * 360
            z = angles[2] * 360

            engagement = compute_engagement_score(x, y, z)

            cv2.putText(img, f"Engagement: {engagement:.3f}", (20, 450), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)

            nose_3d_projection, jacobian = cv2.projectPoints(nose_3d, rot_vec, trans_vec, cam_matrix, dist_matrix)

            p1 = (int(nose_2d[0]), int(nose_2d[1]))
            p2 = (int(nose_2d[0] + y * 10), int(nose_2d[1] - x * 10))

            cv2.line(img, p1, p2, (255, 0, 0), 3)

        mp_drawing.draw_landmarks(
            image=img,
            landmark_list=face_landmarks,
            connections=mp_face_mesh.FACEMESH_CONTOURS,
            landmark_drawing_spec=drawing_spec,
            connection_drawing_spec=drawing_spec
        )


    cv2.imshow("Head Pose Estimation", img)

    if cv2.waitKey(5) & 0xFF == 27:
        break

cap.release() 

