# must run sudo modprobe bcm2835-v4l2

from collections import deque
import time
import imutils
import cv2

import numpy as np

import wall_eecs149


print(wall_eecs149.WALL_EECS149)


# CV2 code inspired by https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
# "Ball Tracking with OpenCV" by Adrian Rosebrock, 9/14/2015

BUFFER_SIZE = 64


# Experimental values
RED_LOWER = (0, 65 * 255 // 100, 15 * 255 // 100)
RED_UPPER = (15 * 2, 80 * 255 // 100, 35 * 255 // 100)

pts = deque(maxlen=BUFFER_SIZE)

camera = cv2.VideoCapture(0)

time_per_frame = 1 / 20
lastUpdate = time.time()
last_ball_time = 0
while True:
    if time.time() - lastUpdate < time_per_frame:
        time.sleep(time_per_frame / 5)
        continue
    lastUpdate = time.time()
    (grabbed, frame) = camera.read()

    frame = imutils.resize(frame, width=600)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, RED_LOWER, RED_UPPER)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
    if len(cnts) > 0 and radius >= 10:
        last_ball_time = time.time()
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        print(center)
        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
    pts.appendleft(center)

    for i in range(1, len(pts)):
        if pts[i - 1] is None or pts[i] is None:
            continue
        thickness = int(np.sqrt(BUFFER_SIZE / float(i + 1)) * 2.5)
        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

camera.release()
cv2.destroyAllWindows()
