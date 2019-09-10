import sys
# parser: rtph264depay
# _parserName = "h264parse";
# _swDecoderName = "avdec_h264";
# for raw video https://stackoverflow.com/questions/39492658/working-example-of-rtpvrawpay-in-gstreamer
# gstreamer tutorial http://www.einarsundgren.se/gstreamer-basic-real-time-streaming-tutorial/
# capture latest frame https://stackoverflow.com/questions/43665208/how-to-get-the-latest-frame-from-capture-device-camera-in-opencv-python#targetText=Your%20program%20should%20be%20able,last%20frame%20in%20the%20queue.&targetText=As%20per%20OpenCV%20reference%2C%20vidcap.read()%20returns%20a%20bool.
# appsink properties https://gstreamer.freedesktop.org/documentation/app/appsink.html?gi-language=c#properties



import cv2
import imutils
import time
#  gst-launch-1.0 -v udpsrc port=5000 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! autovideosink
# gst-launch-1.0 -v  udpsrc port=5600 ! "application/x-rtp, payload=127" ! rtph264depay ! avdec_h264 ! autovideosink
from vision.helpers import Helpers
from vision.shapedetector import ShapeDetector
from shape import Line

# cap = cv2.VideoCapture('udpsrc port=5600 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! avdec_h264 ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
# cap = cv2.VideoCapture('udpsrc port=5600 caps = "application/x-rtp, media=(string)video, encoding-name=(string)H264" ! rtph264depay ! avdec_h264 ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
cap = cv2.VideoCapture('udpsrc port=5600 caps="application/x-rtp, payload=127" ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=true', cv2.CAP_GSTREAMER)


cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)
# cap = cv2.VideoCapture(
#     'udpsrc port=5600 caps = "video/x-raw, format="I420", width',
#     cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print('VideoCapture not opened')
    exit(-1)

while True:
    start_time = time.time()
    # frame = cv2.imread('testpic.png')
    cap.grab()
    ret, frame = cap.retrieve()
    if not ret:
        print('frame empty')
        break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    thresh = cv2.threshold(blurred, 240, 255, cv2.THRESH_BINARY)[1]
    cv2.imshow('threh', thresh)
    # cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # cnts = cnts[0] if imutils.is_cv2() else cnts[1]

    sd = ShapeDetector()
    h = Helpers()


    rows, cols = frame.shape[:2]

    lines = []
    angle1 = []
    angle2 = []
    i = 0

    img_center = (int(cols / 2), int(rows / 2))

    cv2.circle(frame, img_center, 7, (0, 0, 0), -1) # Image center point





    for c in cnts[0]:
        shape = sd.detect(c)
        cv2.drawContours(frame, [c], -1, (255, 0, 0), 1)
        if shape == "rectangle":
            cv2.drawContours(frame, [c], -1, (0, 0, 255), 2)
        else:
            continue
        M = cv2.moments(c)  # https://www.youtube.com/watch?v=AAbUfZD_09s
        if M["m00"] == 0:
            continue
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        # print((i, cX, cY))

        # cv2.circle(frame, (cX, cY), 7, (255, 0, 0), -1) # circle in center of shape
        # cv2.putText(frame, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2) # shape name
        i = i + 1

        [vx, vy, x, y] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
        lefty = int((-x * vy / vx) + y)
        righty = int(((cols - x) * vy / vx) + y)

        point1 = (cols - 1, righty)
        point2 = (0, lefty)

        # cv2.line(img, point1, point2, (0, 255, 0), 2)
        newline = Line((cX, cY), point1, point2, img_center)

        lines.append(newline)
        # plt.plot([cols - 1, righty], [0, lefty], color='k', linestyle='-', linewidth=2)
        # break

    for l in lines:
        try:
            cv2.line(frame, l.p1, l.p2, (0, 255, 0), 2)
        except:
            pass

    end_time = time.time() - start_time
    cv2.putText(frame, "Calc time: " + str(end_time) + " sec", (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 100), 2)

    cv2.imshow('image', frame)

    if cv2.waitKey(1) & 0XFF == ord('q'):
        break

# cap.release()
cv2.destroyAllWindows()
