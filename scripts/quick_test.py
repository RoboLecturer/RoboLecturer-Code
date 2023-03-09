from naoqi import ALProxy, ALBroker
import time

broker = ALBroker("broker", "0.0.0.0", 54000, "192.168.0.104", 9559)
dev = ALProxy("ALTracker")
dev.pointAt("LArm", [1,4,1], 0, 0.5)
# except KeyboardInterrupt:
# 	broker.shutdown()
