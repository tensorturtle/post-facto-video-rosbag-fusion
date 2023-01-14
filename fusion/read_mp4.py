# demonstrate reading mp4 file frame by frame

import cv2
import numpy as np
import sys

cap = cv2.VideoCapture(sys.argv[1])

while(cap.isOpened()):
    ret, frame = cap.read()

    if ret:
        print("Frame shape: ", frame.shape)
    
    else:
        break

