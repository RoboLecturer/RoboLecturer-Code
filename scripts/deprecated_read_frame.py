import sys
from PyQt4.QtGui import QWidget, QImage, QApplication, QPainter
from naoqi import ALProxy
import cv2
import numpy as np
import vision_definitions

class ImageWidget(QWidget):
	"""
	Tiny widget to display camera images from Naoqi.
	"""
	def __init__(self, IP, PORT, CameraID, parent=None):
		"""
		Initialization.
		"""
		QWidget.__init__(self, parent)
		self._image = QImage()
		self.setWindowTitle('Nao')

		self._imgWidth = 320
		self._imgHeight = 240
		self._cameraID = CameraID
		self.resize(self._imgWidth, self._imgHeight)

		# Proxy to ALVideoDevice.
		self._videoProxy = None

		# Our video module name.
		self._imgClient = ""

		# This will contain this alImage we get from Nao.
		self._alImage = None

		self._registerImageClient(IP, PORT)

		# Trigget 'timerEvent' every 100 ms.
		self.startTimer(50)


	def _registerImageClient(self, IP, PORT):
		"""
		Register our video module to the robot.
		"""
		self._videoProxy = ALProxy("ALVideoDevice", IP, PORT)
		resolution = vision_definitions.kQVGA  # 320 * 240
		colorSpace = vision_definitions.kRGBColorSpace
		fps = 10
		self._imgClient = self._videoProxy.subscribe("_client", resolution, colorSpace, fps)

		# Select camera.
		self._videoProxy.setParam(vision_definitions.kCameraSelectID, self._cameraID)


	def _unregisterImageClient(self):
		"""
		Unregister our naoqi video module.
		"""
		if self._imgClient != "":
			self._videoProxy.unsubscribe(self._imgClient)


	def paintEvent(self, event):
		"""
		Draw the QImage on screen.
		"""
		painter = QPainter(self)
		painter.drawImage(painter.viewport(), self._image)
		# img = self.convertQImageToMat(self._image)
		# if img is None:
		# 	return
		# cv2.imshow("Video", img)
		# key = cv2.waitKey(1) & 0xFF

	def convertQImageToMat(self, incomingImage):
		'''  Converts a QImage into an opencv MAT format  '''
		
		incomingImage = incomingImage.convertToFormat(4)
		width = incomingImage.width()
		height = incomingImage.height()

		ptr = incomingImage.bits()
		if ptr is None:
				return None
		ptr.setsize(incomingImage.byteCount())
		arr = np.array(ptr).reshape(height, width, 4)  #  Copies the data
		return arr


	def _updateImage(self):
		"""
		Retrieve a new image from Nao.
		"""
		self._alImage = self._videoProxy.getImageRemote(self._imgClient)
		self._image = QImage(self._alImage[6],			 # Pixel array.
							 self._alImage[0],			 # Width.
							 self._alImage[1],			 # Height.
							 QImage.Format_RGB888)


	def timerEvent(self, event):
		"""
		Called periodically. Retrieve a nao image, and update the widget.
		"""
		self._updateImage()
		self.update()


	def __del__(self):
		"""
		When the widget is deleted, we unregister our naoqi video module.
		"""
		self._unregisterImageClient()
		cv2.destroyAllWindows()
		sys.exit()


IP = "192.168.0.104"  # Replace here with your NaoQi's IP address.
PORT = 9559
CameraID = 0

# Read IP address from first argument if any.
if len(sys.argv) > 1:
	IP = sys.argv[1]

# Read CameraID from second argument if any.
if len(sys.argv) > 2:
	CameraID = int(sys.argv[2])


app = QApplication(sys.argv)
myWidget = ImageWidget(IP, PORT, CameraID)
myWidget.show()
sys.exit(app.exec_())
