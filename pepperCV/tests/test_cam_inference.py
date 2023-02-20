import numpy as np
import pandas as pd
import cv2
from sys import argv, exec_prefix
import os
import threading
import random
import time
from functools import wraps

execution_time = []

def time_inference(func):
    @wraps(func)
    def timeit_wrapper(*args, **kwargs):
        start__inference = time.perf_counter()
        result = func(*args, **kwargs)
        end_inference = time.perf_counter()
        total_time_inference = end_inference - start__inference
        total_time_inference = total_time_inference * 1000
        execution_time.append(total_time_inference)
        print(f'Function {func.__name__} took {total_time_inference:.4f} ms')
        return result
    return timeit_wrapper


cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
cap.set(3, 1920)
cap.set(4, 1080)


@time_inference
def read_video(cap):
    return cap.read()

while True:
    ret, frame = read_video(cap)
    img = frame#np.random.rand(840, 840, 3)#cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # to make frames gray.

    cv2.imshow('video', img)
    k = cv2.waitKey(30) 
    if k == 27: # press 'ESC' to quit
        pd.DataFrame(np.array(execution_time)).to_csv("execution.csv", index=False)
        break

cap.release()
cv2.destroyAllWindows()