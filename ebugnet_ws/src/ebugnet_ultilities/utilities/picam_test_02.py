#!/usr/bin/env python3
# Ahmet 20230409

import cv2 
from datetime import datetime
import time

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)

# Frame dimensions:
# 16:9 - 1296x730, 1920x1080
# 4:3 - 640x480, 1296x972, 2592x1944
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

prev = datetime.now().strftime("%H%M%S%f")
while (True):
    now = datetime.now().strftime("%H%M%S%f")
    fname = "frames/" + now + ".jpg"
    ret, frame = cap.read()
    if ret:
        delta_time = int(now) - int(prev)
        print(delta_time, frame.shape[:2])
        cv2.imwrite(fname, frame)
    else:
        print("Problem: Frame not captured.")
    prev = now
    time.sleep(0.125) 
    
cap.release()
# ---
