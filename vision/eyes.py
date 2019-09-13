# parser: rtph264depay
# _parserName = "h264parse";
# _swDecoderName = "avdec_h264";
# for raw video https://stackoverflow.com/questions/39492658/working-example-of-rtpvrawpay-in-gstreamer
# gstreamer tutorial http://www.einarsundgren.se/gstreamer-basic-real-time-streaming-tutorial/
# capture latest frame https://stackoverflow.com/questions/43665208/how-to-get-the-latest-frame-from-capture-device-camera-in-opencv-python#targetText=Your%20program%20should%20be%20able,last%20frame%20in%20the%20queue.&targetText=As%20per%20OpenCV%20reference%2C%20vidcap.read()%20returns%20a%20bool.
# appsink properties https://gstreamer.freedesktop.org/documentation/app/appsink.html?gi-language=c#properties


import cv2, threading

from helpers.vision import Helpers
from vision.shapedetector import ShapeDetector
from vision.shape import Line


class Eyes:
    def __init__(self, queue):
        self.capture_src = 'udpsrc port=5600 caps="application/x-rtp, payload=127" ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=true'
        self.capture_opts = cv2.CAP_GSTREAMER
        self.cap = None
        self.image_center = (0, 0)
        self.sd = ShapeDetector()
        self.h = Helpers()
        self.image_cols = 0
        self.image_rows = 0
        self.conts = None
        self.lines = None
        self.lines_ahead = None
        self.start_time = 0
        self.roll_drift = 0
        self.yaw_drift = 0
        self.q = queue

    def start_capture(self):
        print("Starting video capture")
        self.cap = cv2.VideoCapture(self.capture_src, self.capture_src)
        if not self.cap.isOpened():
            print("VideoCapture not opened")
            exit(-1)

        if cv2.waitKey(1) & 0XFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()



    def process(self):
        while True:
            self.cap.grab()
            ret, frame = self.cap.retrieve()
            if not ret:
                print('frame empty')
                continue
            self.process_frame(frame)

    def process_image(self):
        frame = cv2.imread('testpic_cross.png')
        self.process_frame(frame)
        while True:
            # do nothing for a while
            if cv2.waitKey(1) & 0XFF == ord('q'):
                self.cap.release()
                cv2.destroyAllWindows()

    def process_frame(self, frame):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        thresh = cv2.threshold(blurred, 240, 255, cv2.THRESH_BINARY)[1]
        # cv2.imshow('threh', thresh)
        self.conts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        rows, cols = frame.shape[:2]  # frame dimensions
        self.image_cols = cols
        self.image_rows = rows

        self.img_center = (int(cols / 2), int(rows / 2))

        self.process_contours()
        self.calculate_roll_drift()
        self.calculate_yaw_drift()
        self.draw_image(frame)

    def process_contours(self):
        sd = ShapeDetector()
        lines = []
        for c in self.conts[0]:
            shape = sd.detect(c)
            if shape == "rectangle":
                moments = cv2.moments(c)  # get rectangle X and Y axis -  https://www.youtube.com/watch?v=AAbUfZD_09s
                if moments["m00"] == 0:
                    continue
                # calculate center point of rectangle
                cx = int(moments["m10"] / moments["m00"])
                cy = int(moments["m01"] / moments["m00"])
            else:
                continue

            [vx, vy, x, y] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
            lefty = int((-x * vy / vx) + y)
            righty = int(((self.image_cols - x) * vy / vx) + y)

            point1 = (self.image_cols - 1, righty)
            point2 = (0, lefty)

            newline = Line((cx, cy), point1, point2, self.img_center)
            lines.append(newline)
        self.lines = lines

    def calculate_roll_drift(self):
        lines = self.lines

        if len(lines) < 2:
            print("Not enough lines found, nothing to do here")
            return

        lines.sort(key=lambda l: l.centerdistance)
        roll_line = Line(lines[0].guidepoint, lines[0].guidepoint, lines[1].guidepoint, self.img_center)
        cross_line = roll_line.plot_point(self.img_center, int(roll_line.angle) + 90, 250)
        cross_point = Line.line_intersection((roll_line.p1, roll_line.p2), (cross_line[0], cross_line[1]))
        self.roll_drift = round(self.h.distance(cross_point, self.img_center), 0)

    def calculate_yaw_drift(self):
        lines = self.lines
        lines_ahead = [l for l in lines if l.guidepoint[1] <= self.img_center[1]]
        self.lines_ahead = lines_ahead

        if len(lines_ahead) < 3:
            print("No ahead lines found, nothing todo here ")
            return

        lines_ahead.sort(key=lambda l: l.centerdistance)
        best_course = Line(lines_ahead[0].guidepoint, lines_ahead[0].guidepoint, lines_ahead[1].guidepoint,
                           self.img_center)
        yaw_drift = round(best_course.angle, 0)

        if 360 > yaw_drift > 270:
            # if best course is to right, give negative angle from 0 deg
            yaw_drift = yaw_drift - 360

        self.yaw_drift = yaw_drift

    def draw_image(self, frame):
        cv2.drawContours(frame, [self.conts[0]], -1, (255, 0, 0), 1)
        cv2.circle(frame, self.lines_ahead[0].guidepoint, 7, (0, 255, 0), -1)  # closest point to center of image
        cv2.circle(frame, self.lines_ahead[1].guidepoint, 7, (255, 255, 0), -1)  # ... one above it
        cv2.circle(frame, self.lines_ahead[2].guidepoint, 7, (255, 255, 0), -1)  # ... and one below

        cv2.putText(frame, "Roll drift: " + str(self.roll_drift) + " px", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 100), 2)

        cv2.putText(frame, "Yaw drift: " + str(self.yaw_drift) + " deg", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 100), 2)
        cv2.imshow('image', frame)

