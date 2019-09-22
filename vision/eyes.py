# parser: rtph264depay
# _parserName = "h264parse";
# _swDecoderName = "avdec_h264";
# for raw video https://stackoverflow.com/questions/39492658/working-example-of-rtpvrawpay-in-gstreamer
# gstreamer tutorial http://www.einarsundgren.se/gstreamer-basic-real-time-streaming-tutorial/
# capture latest frame https://stackoverflow.com/questions/43665208/how-to-get-the-latest-frame-from-capture-device-camera-in-opencv-python#targetText=Your%20program%20should%20be%20able,last%20frame%20in%20the%20queue.&targetText=As%20per%20OpenCV%20reference%2C%20vidcap.read()%20returns%20a%20bool.
# appsink properties https://gstreamer.freedesktop.org/documentation/app/appsink.html?gi-language=c#properties


import cv2, math

from helpers.vision import Helpers
from vision.shapedetector import ShapeDetector
from vision.shape import Line
import numpy as np


class Eyes:
    def __init__(self, queue):
        self.capture_src = 'udpsrc port=5600 caps="application/x-rtp, payload=127" ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=true'
        # self.capture_src = 'udpsrc port="5600" caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)RAW, sampling=(string)YCbCr-4:2:0, depth=(string)8, width=(string)720, height=(string)1280, colorimetry=(string)BT601-5, payload=(int)96, ssrc=(uint)1103043224, timestamp-offset=(uint)1948293153, seqnum-offset=(uint)27904" ! rtpvrawdepay ! videoconvert ! queue ! appsink sync=false'
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
        self.roll_line = None
        self.best_course = None
        self.start_time = 0
        self.roll_drift = 0
        self.yaw_drift = 0
        self.q = queue
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.heading = 0

        self.north = []
        self.east = []
        self.speed = 0
        self.altitude = 0
        self.meters_front = 0
        self.pixels_per_m = 0
        self.velocity_line = None
        self.moving_line = None
        self.drive_dir = None

    def start_capture(self):
        print("Starting video capture")
        self.cap = cv2.VideoCapture(self.capture_src, self.capture_opts)
        if not self.cap.isOpened():
            print("VideoCapture not opened")
            exit(-1)

        while True:
            # get some pyshical attributes
            data = self.q.get()

            self.pitch = data.pitch
            self.roll = data.roll
            self.yaw = data.yaw
            self.altitude = data.altitude
            self.speed = data.speed

            # update location only when moving
            if self.speed >= 0.1:
                self.north.append(data.north)
                self.east.append(data.east)

            self.cap.grab()
            ret, image = self.cap.retrieve()
            if not ret:
                print('frame empty')
                continue
            frame = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
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
        # cv2.imshow('blurred', blurred)
        thresh = cv2.threshold(blurred, 240, 255, cv2.THRESH_BINARY)[1]
        # cv2.imshow('threh', thresh)
        self.conts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        rows, cols = frame.shape[:2]  # frame dimensions
        self.image_cols = cols
        self.image_rows = rows

        self.image_center = (int(cols / 2), int(rows / 2))

        # figure out flying direction (only if enough samples exists
        if len(self.north) > 8 and len(self.east) > 8:
            self.velocity_line = Line(self.image_center, (self.north[0], self.east[0]), (self.north[-1], self.east[-1]), self.image_center)
            self.velocity_line.angle = self.velocity_line.angle
            self.heading = self.velocity_line.angle

            velocity_point = self.velocity_line.plot_point(self.image_center, self.velocity_line.angle - 90 - self.yaw, self.speed * 200 + 20)
            self.moving_line = Line(self.image_center, velocity_point[1], self.image_center, self.image_center)



        # Calculate and process all kind of stuff
        self.process_contours()
        self.calculate_roll_drift()
        self.calculate_yaw_drift()
        self.calculate_meters_in_view()

        # reset flight path history
        if len(self.north) >= 10:
            self.north.pop(0)

        if len(self.east) >= 10:
            self.east.pop(0)

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

            newline = Line((cx, cy), point1, point2, self.image_center)
            if self.moving_line is not None and abs(newline.angle) - abs(self.moving_line.angle) < 20:
                lines.append(newline)
        self.lines = lines

    def calculate_roll_drift(self):
        lines = self.lines

        if len(lines) < 2:
            # print("Not enough lines found, nothing to do here")
            self.roll_line = None
            self.roll_drift = 0
            return

        lines.sort(key=lambda l: l.centerdistance)
        roll_line = Line(lines[0].guidepoint, lines[0].guidepoint, lines[1].guidepoint, self.image_center)
        cross_line = roll_line.plot_point(self.image_center, int(roll_line.angle) + 90, 250)
        cross_point = Line.line_intersection((roll_line.p1, roll_line.p2), (cross_line[0], cross_line[1]))
        self.roll_line = roll_line
        self.roll_drift = round(self.h.distance(cross_point, self.image_center), 0)

    def calculate_yaw_drift(self):
        lines = self.lines
        lines_ahead = [l for l in lines if l.guidepoint[1] <= self.image_center[1]]
        self.lines_ahead = lines_ahead

        if len(lines_ahead) < 3:
            # print("No ahead lines found, nothing todo here ")
            self.yaw_drift = 0
            self.best_course = None
            return

        lines_ahead.sort(key=lambda l: l.centerdistance)
        best_course = Line(lines_ahead[0].guidepoint, lines_ahead[0].guidepoint, lines_ahead[1].guidepoint,
                           self.image_center)
        yaw_drift = round(best_course.angle, 0)

        if 360 > yaw_drift > 270:
            # if best course is to right, give negative angle from 0 deg
            yaw_drift = yaw_drift - 360

        self.best_course = best_course
        self.yaw_drift = yaw_drift

    def calculate_meters_in_view(self):
        self.meters_front = self.altitude * math.atan(85 / 2)
        self.pixels_per_m = self.image_rows / 2 / self.meters_front

    def draw_image(self, frame):
        cv2.circle(frame, self.image_center, 7, (200, 100, 255), -1)  # Image center point
        try:
            cv2.drawContours(frame, self.conts[0], -1, (255, 0, 0), 1)
        except:
            pass

        # if self.lines_ahead is not None and len(self.lines_ahead) >= 3:
        try:
            cv2.circle(frame, self.lines_ahead[0].guidepoint, 7, (0, 255, 0), -1)
        except:
            pass
        try:
            cv2.circle(frame, self.lines_ahead[1].guidepoint, 7, (255, 255, 0), -1)
        except:
            pass
        try:
            cv2.circle(frame, self.lines_ahead[2].guidepoint, 7, (255, 255, 0), -1)
        except:
            pass

        try:
            cv2.line(frame, self.roll_line.p1, self.roll_line.p2, (0, 255, 255), 2)
        except:
            pass
        try:
            cv2.line(frame, self.best_course.p1, self.best_course.p2, (0, 255, 0), 2)
        except:
            pass

        try:
            cv2.putText(frame, "Pitch " + str(round(self.pitch, 3)) + " deg", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 100), 2)
        except:
            pass

        try:
            cv2.putText(frame, "Roll " + str(round(self.roll, 3)) + " deg", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 100), 2)
        except:
            pass

        try:
            cv2.putText(frame, "Yaw " + str(round(self.yaw, 3)) + " deg", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 100), 2)
        except:
            pass

        try:
            cv2.putText(frame, "heading " + str(round(self.heading, 3)) + " deg", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 100), 2)
        except:
            pass



        try:
            cv2.putText(frame, "Altitude " + str(round(self.altitude, 3)) + " m", (10, 130), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 100), 2)
        except:
            pass

        try:
            cv2.putText(frame, "Distance front " + str(round(self.meters_front, 3)) + " m", (10, 150),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 100), 2)
        except:
            pass

        try:
            cv2.putText(frame, "pix/m " + str(round(self.pixels_per_m, 3)) + " px", (10, 170),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 100), 2)
        except:
            pass

        try:
            cv2.putText(frame, "north " + str(self.north[-1]), (10, 190),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 100), 2)
        except:
            pass

        try:
            cv2.putText(frame, "east " + str(self.east[-1]), (10, 210),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 100), 2)
        except:
            pass

        try:
            cv2.putText(frame, "Speed " + str(round(self.speed, 3)) + ' m/s', (10, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 100), 2)
        except:
            pass


        try:
            cv2.line(frame, self.moving_line.p1, self.moving_line.p2, (0, 255, 0), 2)
        except:
            pass

        cv2.imshow('image', frame)
        if cv2.waitKey(1) & 0XFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
