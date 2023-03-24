import mediapipe as mp
import cv2
import numpy as np
import time
import random

def landmark_model():
    return mp.solutions.face_mesh.FaceMesh(max_num_faces=3, min_detection_confidence=0.2, min_tracking_confidence=0.5)


def project_landmarks(landmarks, frame, face_2d, face_3d):
    indices = [33, 263, 1, 61, 291, 199]
    img_h, img_w, img_c = frame.shape

    if landmarks.multi_face_landmarks:
        for face_landmarks in landmarks.multi_face_landmarks:
            for idx, lm in enumerate(face_landmarks.landmark):
                if idx in indices:
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

            return x, y, z

    return 0, 0, 0


def engagement_from_landmarks(landmarks, frame, w, h):
    img_h, img_w, img_c = frame.shape
    if landmarks.any():
        for landmark in landmarks:
            z = random.uniform(-1, 1)
            for i in range(0, landmarks.shape[1] -1, 2):
                if i == 6:
                    x_nose = int(landmark[i])
                    y_nose = int(landmark[i + 1])

                x = int(landmark[i])
                y = int(landmark[i + 1])
                cv2.circle(frame, (x, y), radius=2, color=(0, 255, 0), thickness=2)

            p_nose = np.array([x_nose, y_nose])
            p_nose_norm = np.divide(p_nose, np.divide(h, w))
            similarity_nose = 1.0/np.cosh(0.002 * p_nose_norm[0]) * 1.0/np.cosh(0.002 * p_nose_norm[1])

            return similarity_nose

    return 0


def compute_engagement_score(x, y, z):
    if x == 0 and y == 0 and z == 0:
        return 0.0
    return 1.0/(np.cosh(0.12*x)) * 1.0/(np.cosh(0.12*y))# * 1.0/(np.cosh(0.12*z))

if __name__ == "__main__":
    mp_face_mesh = mp.solutions.face_mesh
    face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5)
    mp_drawing = mp.solutions.drawing_utils
    drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)


    cap = cv2.VideoCapture("/dev/video0")
    cap.set(3, 1920)
    cap.set(4, 1080)

    while cap.isOpened():
        ret, img = cap.read()
        print(f"Image: {img}")

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

