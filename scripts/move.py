import os
from naoqi import ALProxy
import time
motion = ALProxy("ALMotion", os.environ["NAO_IP"], 9559)
motion.setExternalCollisionProtectionEnabled("All", False)
motion.move(0,0.15,0)
time.sleep(3)
print("done")
motion.move(0,-0.15,0)
time.sleep(3)
motion.move(0,0,0)
print("done2")
motion.setExternalCollisionProtectionEnabled("All", True)
