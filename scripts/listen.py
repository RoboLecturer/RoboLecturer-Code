import PepperAPI
from PepperAPI import Action, Info
import threading

PepperAPI.init("master")
t1 = threading.Thread(target=Action.Listen)
t2 = threading.Thread(target=Info.Listen)
t1.start()
t2.start()

try:
	while True:
		pass
except KeyboardInterrupt:
	Action.KillThreads()
	Info.KillThreads()
	t1.join()
	t2.join()
