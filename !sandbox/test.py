import cv2
import numpy as np

cv2.namedWindow('image', cv2.WINDOW_NORMAL)
# cv2.resizeWindow('image', (2600,2600))
# cv2.setWindowProperty('image', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN);

cap = cv2.VideoCapture(0)

# Check if the webcam is opened correctly
if not cap.isOpened():
    raise IOError("Cannot open webcam")

while True:
    ret, image = cap.read()
    frame = cv2.resize(image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    thresh = cv2.threshold(frame, 130, 200, cv2.THRESH_BINARY)[1]

    vis = np.concatenate((frame, thresh), axis=1)
    cv2.imshow('image', vis)
    # cv2.imshow('thresh', thresh)
    # cv2.moveWindow('image', 0, 0)
    # cv2.moveWindow('thresh', 700, 0)
    c = cv2.waitKey(1)
    if c == 27:
        break

cap.release()
cv2.destroyAllWindows()
