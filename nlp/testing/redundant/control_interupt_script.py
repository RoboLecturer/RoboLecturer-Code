import threading 
import signal

def my_thread():
    while not stop_thread:
        continue


def signal_handler(sig,frame):
    global stop_thread
    print("Leyboard interrupt recieved. Stopping thread...")
    stop_thread = True

signal.signal(signal.SIGUSR1, signal_handler)

stop_thread = False 

thread = threading.Thread(target=my_thread)
thread.start()

thread.join()


