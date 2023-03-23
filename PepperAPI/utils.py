import cv2
import numpy as np

"""x-offset from world center in cm"""
# MARKER_DIST = {
# 	5: -140,
# 	10: -70,
# 	15: 0,
# 	20: 70,
# 	25: 140
# }
MARKER_DIST = {
	5: -70,
	10: -0,
	15: 70
}

def pose_estimation(frame, (width,height), aruco_dict_type, matrix_coefficients, distortion_coefficients):

	'''
	frame - Frame from the video stream
	matrix_coefficients - Intrinsic matrix of the calibrated camera
	distortion_coefficients - Distortion coefficients associated with your camera

	return:-
	frame - The frame with the axis drawn on it
	'''

	global MARKER_DIST

	def get_origins(z):
		# Pepper QVGA
		x1, y1, z1 = -80.0, -65.0, 320.0
		x2, y2, z2 = -160.0, -125.0, 640.0
		x0 = (x2 - x1) / (z2 - z1) * (z - z1) + x1
		y0 = (y2 - y1) / (z2 - z1) * (z - z1) + y1
		return x0, y0

	def get_offset_from_marker_center(x, y, z):
		"""return distance from center in cm"""
		px_to_cm = 10.0 / 20 # for Pepper QVGA. 10.0cm ~ 23px
		x0, y0 = get_origins(z)
		# print(int(x0), int(y0), int(z))
		offset_x = (x - x0) * px_to_cm
		offset_y = (y - y0) * px_to_cm
		return offset_x, offset_y

	def get_offset_from_world_center(marker_id, dx):
		"""return offset from CV camera center in px"""
		cm_to_px = 337.0 / 100 # for CV camera. 337px ~ 100cm
		if marker_id not in MARKER_DIST:
			return 0
		D = (MARKER_DIST[marker_id] - dx) * cm_to_px
		return int(D)

	
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
	parameters = cv2.aruco.DetectorParameters_create()

	corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters)
		
	Dlst = []
	T = [0,0,0]
	# If markers are detected
	if len(corners) > 0:
		# print(corners)
		for i in range(0, len(ids)):
			marker_id = int(ids[i])
			# Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
			rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients, distortion_coefficients)
			
			# Draw a square around the markers
			cv2.aruco.drawDetectedMarkers(frame, corners) 

			# Draw Axis
			cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.001)

			T = tvec[0][0] * 1000	# pepper QVGA
			offset_x, offset_y = get_offset_from_marker_center(T[0], T[1], T[2])
			D = get_offset_from_world_center(marker_id, offset_x)
			Dlst.append(D)

	D = np.mean(Dlst) if len(Dlst) else None
	cv2.circle(frame, (width//2, height//2), 1, (255,0,0), 2)
	cv2.line(frame, (width//2, 0), (width//2, height), (255,0,0), 1)
	cv2.line(frame, (0, height//2), (width, height//2), (255,0,0), 1)
	return frame, D


