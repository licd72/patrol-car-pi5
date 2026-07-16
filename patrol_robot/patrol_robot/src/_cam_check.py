#!/usr/bin/env python3
import cv2
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
ret, frame = cap.read()
cap.release()
if ret and frame is not None:
    print(f"CAM_OK {frame.shape}")
    exit(0)
else:
    exit(1)
