# parser: rtph264depay
# _parserName = "h264parse";
# _swDecoderName = "avdec_h264";
# for raw video https://stackoverflow.com/questions/39492658/working-example-of-rtpvrawpay-in-gstreamer
# gstreamer tutorial http://www.einarsundgren.se/gstreamer-basic-real-time-streaming-tutorial/
# capture latest frame https://stackoverflow.com/questions/43665208/how-to-get-the-latest-frame-from-capture-device-camera-in-opencv-python#targetText=Your%20program%20should%20be%20able,last%20frame%20in%20the%20queue.&targetText=As%20per%20OpenCV%20reference%2C%20vidcap.read()%20returns%20a%20bool.
# appsink properties https://gstreamer.freedesktop.org/documentation/app/appsink.html?gi-language=c#properties



import cv2
import time
#  gst-launch-1.0 -v udpsrc port=5000 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! autovideosink
# gst-launch-1.0 -v  udpsrc port=5600 ! "application/x-rtp, payload=127" ! rtph264depay ! avdec_h264 ! autovideosink
from helpers.vision import Helpers
from vision.shapedetector import ShapeDetector
from vision.shape import Line

# cap = cv2.VideoCapture('udpsrc port=5600 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! avdec_h264 ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
# cap = cv2.VideoCapture('udpsrc port=5600 caps = "application/x-rtp, media=(string)video, encoding-name=(string)H264" ! rtph264depay ! avdec_h264 ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
# cap = cv2.VideoCapture('udpsrc port=5600 caps="application/x-rtp, payload=127" ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=true', cv2.CAP_GSTREAMER)


# cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)
# cap = cv2.VideoCapture(
#     'udpsrc port=5600 caps = "video/x-raw, format="I420", width',
#     cv2.CAP_GSTREAMER)
# if not cap.isOpened():
#     print('VideoCapture not opened')
#     exit(-1)

while True:
    start_time = time.time()
    frame = cv2.imread('testpic_cross.png')
    # cap.grab()
    # ret, frame = cap.retrieve()
    # if not ret:
    #     print('frame empty')
    #     break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    thresh = cv2.threshold(blurred, 240, 255, cv2.THRESH_BINARY)[1]
    # cv2.imshow('threh', thresh)
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    sd = ShapeDetector()
    h = Helpers()

    rows, cols = frame.shape[:2] # frame dimensions

    lines = []
    angle1 = []
    angle2 = []
    i = 0

    img_center = (int(cols / 2), int(rows / 2))

    cv2.circle(frame, img_center, 7, (100, 100, 100), -1) # Image center point

    for c in cnts[0]:
        shape = sd.detect(c)
        cv2.drawContours(frame, [c], -1, (255, 0, 0), 1)
        if shape == "rectangle":
            M = cv2.moments(c)  # get rectangle X and Y axis -  https://www.youtube.com/watch?v=AAbUfZD_09s
            if M["m00"] == 0:
                continue
            # calculate center point of rectangle
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(frame, (cX, cY), 7, (0, 0, 255), -1) # circle in center of shape
        else:
            continue

        i = i + 1

        [vx, vy, x, y] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
        lefty = int((-x * vy / vx) + y)
        righty = int(((cols - x) * vy / vx) + y)

        point1 = (cols - 1, righty)
        point2 = (0, lefty)

        newline = Line((cX, cY), point1, point2, img_center)
        # cv2.line(frame, newline.p1, newline.p2, (0, 255, 0), 2)  # Draw discovered line


        lines.append(newline)
        # break



    if len(lines) < 2:
        #not enough lines, nothing todo here
        continue


    ## CALCULATE YAW DRIFT
    # get best based on two neared points ahead

    # use only lines ahead of current position
    lines_ahead = [l for l in lines if l.guidepoint[1] <= img_center[1]]

    # sort lines by distance from frame center
    lines_ahead.sort(key=lambda l: l.centerdistance)

    # drown quidepoints of intresting lines
    cv2.circle(frame, lines_ahead[0].guidepoint, 7, (0, 255, 0), -1)  # closest point to center of image
    cv2.circle(frame, lines_ahead[1].guidepoint, 7, (255, 255, 0), -1)  # ... one above it
    cv2.circle(frame, lines_ahead[2].guidepoint, 7, (255, 255, 0), -1)  # ... and one below


    # draw best course line (line between next block and one after that, ahead of current pos)
    best_course = Line(lines_ahead[0].guidepoint, lines_ahead[0].guidepoint, lines_ahead[1].guidepoint, img_center)
    cv2.line(frame, best_course.p1, best_course.p2, (0, 255, 0), 2)  # Draw ideal flight line

    # draw current heading just for reference
    current_direction = Line(img_center, img_center, (int(cols/2), 0), img_center)
    cv2.line(frame, current_direction.p1, current_direction.p2, (100, 200, 100), 2)
    yaw_drift = round(best_course.angle, 0)

    if yaw_drift < 260 and yaw_drift > 270:
        # if best cours is to right, give negative angle from 0 deg
        yaw_drift = yaw_drift - 360
    ## END OF YAW

    ## CALCULATE ROLL DRIFT
    # based on two closed points to center

    # sort by distance from image center
    lines.sort(key=lambda l: l.centerdistance)

    cv2.circle(frame, lines[0].guidepoint, 7, (0, 255, 255), -1)  # closest point to center of image
    cv2.circle(frame, lines[1].guidepoint, 7, (0, 255, 255), -1)  # ... one above it

    # fit line trough
    roll_line = Line(lines[0].guidepoint, lines[0].guidepoint, lines[1].guidepoint, img_center)
    cv2.line(frame, roll_line.p1, roll_line.p2, (0, 255, 255), 2)  # Draw it

    # Line perpendicular to roll line
    cross_line = roll_line.plot_point(img_center, int(roll_line.angle) + 90, 250)

    # closest point on roll line to image center
    cross_point = Line.line_intersection((roll_line.p1, roll_line.p2), (cross_line[0], cross_line[1]))
    cv2.circle(frame, cross_point, 4, (255, 0, 255), -1)  # cross line endpoint

    roll_drift = round(h.distance(cross_point, img_center), 0)

    ## END OF ROLL DRIFT




    cv2.putText(frame, "Roll drift: " + str(roll_drift) + " px", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 100), 2)

    cv2.putText(frame, "Yaw drift: " + str(yaw_drift) + " deg", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 100), 2)


    end_time = time.time() - start_time
    cv2.putText(frame, "Calc time: " + str(round(end_time, 4)) + " sec", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 100), 2)

    cv2.imshow('image', frame)

    if cv2.waitKey(1) & 0XFF == ord('q'):
        break

# cap.release()
cv2.destroyAllWindows()