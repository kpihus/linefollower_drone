import cv2
import imutils
import time
#  gst-launch-1.0 -v udpsrc port=5000 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! autovideosink
from vision.helpers import Helpers
from vision.shapedetector import ShapeDetector
from shape import Line

cap = cv2.VideoCapture('udpsrc port=5600 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
# cap = cv2.VideoCapture(
#     'udpsrc port=5600 caps = "video/x-raw, format="I420", width',
#     cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print('VideoCapture not opened')
    exit(-1)

while True:
    start_time = time.time()
    ret, frame = cap.read()
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


        cv2.circle(frame, (cX, cY), 7, (255, 0, 0), -1)
        cv2.putText(frame, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        i = i + 1

        [vx, vy, x, y] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
        lefty = int((-x * vy / vx) + y)
        righty = int(((cols - x) * vy / vx) + y)

        point1 = (cols - 1, righty)
        point2 = (0, lefty)

        newline = Line((cX, cY), point1, point2, img_center)

        lines.append(newline)

    lines.sort(key=lambda l: l.angle)

    bestline = Line((int(cols / 2), int(rows / 2)), (0, int(rows / 2)), (cols, int(rows / 2)), img_center) # Initial best line is worst ever, 90 degrees with flight path

    # find all good lines (closest to Y axis)
    goodlines = []
    for l in lines:
        if l.angle < abs(bestline.angle):
            goodlines.append(l)

    if len(goodlines) < 3:
        continue

    del goodlines[0]
    del goodlines[-1]


    bestAngle = h.averageLineDeg(goodlines)

    centerline = Line((int(cols / 2), int(rows / 2)), (int(cols / 2), 0), (int(cols / 2), rows), img_center)

    closest_to_c = h.closest_point_to((int(cols / 2), int(rows / 2)), goodlines)

    if len(goodlines) < 3:
        continue

    goodlines.sort(key=lambda l: l.centerdistance)
    cv2.circle(frame, goodlines[0].guidepoint, 7, (0, 0, 255), -1)  # Most center point
    cv2.circle(frame, goodlines[1].guidepoint, 7, (0, 255, 0), -1)  # ... one above it
    cv2.circle(frame, goodlines[2].guidepoint, 7, (0, 255, 0), -1)  # ... and one below

    flight_line = Line(goodlines[0].guidepoint, goodlines[2].guidepoint, goodlines[1].guidepoint, img_center)

    cross_line = flight_line.plot_point(img_center, int(flight_line.angle) + 90, 250)
    cv2.circle(frame, cross_line[1], 4, (255, 0, 255), -1)  # cross line endpoint


    cv2.line(frame, flight_line.p1, flight_line.p2, (0, 255, 0), 2) # Draw ideal flight line
    cv2.line(frame, cross_line[0], cross_line[1], (0, 10, 0), 2) # Line 90deg to flitht line

    cross_point = Line.line_intersection((flight_line.p1, flight_line.p2), (cross_line[0], cross_line[1]))
    cv2.circle(frame, cross_point, 4, (255, 0, 255), -1)  # cross line endpoint

    drift_from_ideal = h.distance(cross_point, img_center)




    cv2.putText(frame, "Yaw drift: "+str(flight_line.angle) + " deg", (100, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 100), 2)
    cv2.putText(frame, "Roll drift: "+str(drift_from_ideal) + " px", (100, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 100), 2)




    end_time = time.time() - start_time
    cv2.putText(frame, "Calc time: " + str(end_time) + " sec", (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 100), 2)

    # Do some stuff here what takes lets say 1sec
    cv2.imshow('image', frame)

    if cv2.waitKey(1) & 0XFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
