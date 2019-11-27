import numpy as np
import cv2 as cv
import argparse
import time
import zmq
import base64

from picamera.array import PiRGBArray
from picamera import PiCamera

context = zmq.Context()
footage_socket = context.socket(zmq.PUB)
footage_socket.connect('tcp://localhost:5555')

# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )
# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
# Create some random colors
color = np.random.randint(0,255,(100,3))
# Take first frame and find corners in it


camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
raw_capture = PiRGBArray(camera, size=(640, 480))
time.sleep(1)

print("getting first frame")
camera.capture(raw_capture, format="bgr")
image = raw_capture.array
raw_capture.truncate(0)

old_gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
p0 = cv.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
# Create a mask image for drawing purposes
mask = np.zeros_like(image)
count = 0

print("Starting opt flow")

for image in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
    start = time.time()
    frame = image.array
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # calculate optical flow
    p1, st, err = cv.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
    # Select good points
    good_new = p1[st==1]
    good_old = p0[st==1]
    # draw the tracks
    for i,(new,old) in enumerate(zip(good_new, good_old)):
        a,b = new.ravel()
        c,d = old.ravel()
        mask = cv.line(mask, (a,b),(c,d), color[i].tolist(), 2)
        frame = cv.circle(frame,(a,b),5,color[i].tolist(),-1)
    img = cv.add(frame,mask)
    encoded, buffer = cv.imencode('.jpg', img)
    jpg_as_text = base64.b64encode(buffer)
    footage_socket.send(jpg_as_text)
    #out.write(img)
    k = cv.waitKey(30) & 0xff
    if k == 27:
        break
    # Now update the previous frame and previous points
    old_gray = frame_gray.copy()
    p0 = good_new.reshape(-1,1,2)
    print(float(time.time() - start))
    raw_capture.truncate(0)