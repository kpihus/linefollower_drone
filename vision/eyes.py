import os
if os.getenv('PLATFORM') == 'BIRD':
    from picamera.array import PiRGBArray
    from picamera import PiCamera

import cv2
import math
import time

import numpy as np
from math import atan2, degrees
import base64
import zmq

from helpers.vision import Helpers
from vision.shape import Line
from vision.shapedetector import ShapeDetector
from models.data import FlightCommands

FONT = cv2.FONT_HERSHEY_SIMPLEX

class Eyes:
    def __init__(self, flight_params = None, flight_commands = None):
        self.capture_src = 'udpsrc buffer-size=24000000 port=5600 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)RAW, sampling=YCbCr-4:2:0,depth=(string)16,width=(string)640, height=(string)640,colorimetry=(string)BT601-5, payload=(int)96, a-framerate=60/1" ! rtpvrawdepay ! videoconvert ! queue ! appsink drop=true'
        # self.capture_src = 'udpsrc port="5600" caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)RAW, sampling=(string)YCbCr-4:2:0, depth=(string)8, width=(string)720, height=(string)1280, colorimetry=(string)BT601-5, payload=(int)96, ssrc=(uint)1103043224, timestamp-offset=(uint)1948293153, seqnum-offset=(uint)27904" ! rtpvrawdepay ! videoconvert ! queue ! appsink sync=false'
        self.capture_opts = cv2.CAP_GSTREAMER
        self.img_thres = None
        self.cap = None
        self.image_center = (0, 0)
        self.sd = ShapeDetector()
        self.h = Helpers()
        self.image_cols = 0
        self.image_rows = 0
        self.thres_val = int(os.getenv('THRES_VAL'))
        self.thres_max = int(os.getenv('THRES_MAX'))
        self.conts = None
        self.lines = None
        self.bad_lines = None
        self.lines_ahead = None
        self.roll_line = None
        self.best_course = None
        self.start_time = 0
        self.roll_drift = 0
        self.correct_yaw = 0
        self.yaw_drift = 0
        self.fpq = flight_params
        self.fcq = flight_commands
        self.flight_params_time = time.time()
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

        self.points1 = []

        context = zmq.Context()
        self.footage_socket = context.socket(zmq.PUB)
        self.footage_socket.connect('tcp://localhost:5555')


    def start_capture(self):
        print('The beginning')
        if os.getenv('PLATFORM') == 'SIMU':
            self.capture_simu()
        elif os.getenv('PLATFORM') == 'BIRD':
            self.capture_pi()
            pass
        else:
            print('Unknown platform')
            exit()

    def capture_pi(self):
        camera = PiCamera()
        camera.resolution = (640, 480)
        camera.framerate = 32
        raw_capture = PiRGBArray(camera)
        time.sleep(0.3)

        # while True:
        #     camera.capture(raw_capture, format="bgr")
        #     image = raw_capture.array
        #     #frame = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        #     self.process_frame(image)
        #     raw_capture.truncate(0)

        for image in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
            self.flight_info()
            frame = image.array
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            self.process_frame(frame)
            #cv2.imshow('orig', frame)
            raw_capture.truncate(0)

    def capture_simu(self):
        print("Starting video capture")
        self.cap = cv2.VideoCapture(self.capture_src, self.capture_opts)
        print("got cap")
        if not self.cap.isOpened():
            print("VideoCapture not opened")
            exit(-1)
        print("Cap is opened")

        while True:
            print("Got frame")
            self.flight_info()

            self.cap.grab()

            ret, image = self.cap.retrieve()
            if not ret:
                print('frame empty')
                continue
            # rotate image 90 deg, because of landscape camera on drone
            frame = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            self.process_frame(frame)

    def flight_info(self):
        # get some pyshical attributes
        if not (self.fpq is None) and not self.fpq.empty():
            data = self.fpq.get()
            if data.timestamp > self.flight_params_time:
                self.flight_params_time = data.timestamp
                self.pitch = data.pitch
                self.roll = data.roll
                self.yaw = data.yaw
                self.altitude = data.altitude
                self.speed = data.speed
                # self.thres_val = self.h.map_values(data.thres_val, inMin=1000, inMax=2000, outMin=0, outMax=255) \
                #     if not (data.thres_val is None) and data.thres_val > 0 else self.thres_val
                # self.thres_max = self.h.map_values(data.thres_max, inMin=1000, inMax=2000, outMin=0, outMax=255) \
                #     if not (data.thres_max is None) and data.thres_max > 0 else self.thres_max

                # update location only when moving
                if self.speed >= 0.1:
                    self.north.append(data.north)
                    self.east.append(data.east)

    def process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.threshold(blurred, self.thres_val, self.thres_max, cv2.THRESH_BINARY)[1]
        self.conts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        rows, cols = frame.shape[:2]  # frame dimensions
        self.image_cols = cols
        self.image_rows = rows

        self.image_center = (int(cols / 2), int(rows / 2))

        # figure out flying direction (only if enough samples exists
        if len(self.north) > 10 and len(self.east) > 10:
            fstart = (self.north[0], self.east[0])
            fend = (self.north[-1], self.east[-1])
            xDiff = fend[0] - fstart[0]
            yDiff = fend[1] - fstart[1]

            angle = degrees(atan2(yDiff, xDiff))

            self.heading = angle - self.yaw
            if self.heading < -180:
                self.heading += 360


        # Calculate and process all kind of stuff
        self.process_contours()
        self.calculate_roll_drift()
        self.calculate_yaw_drift()
        self.calculate_meters_in_view()

        # print("Rolldrift " + str(self.roll_drift)+ " | Yaw drift " + str(self.yaw_drift))

        # reset flight path history
        if len(self.north) >= 20:
            self.north.pop(0)

        if len(self.east) >= 20:
            self.east.pop(0)
        self.fcq.put(FlightCommands(time.time(), self.yaw_drift, self.roll_drift))
        self.draw_image(frame)

    def process_contours(self):
        sd = ShapeDetector()
        lines = []
        bad_lines = []
        points1 = []
        for c in self.conts[1]:
            shape = sd.detect(c)
            print("Shape", shape)
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

            # Point all line to northish direction

            vxa = -vy
            vya = vx
            if vxa < 0:
                vxa = -vxa
                vya = -vya

            # if vxa > 0 and vya > 0:
            #     vya = vya * -1
            #
            # if vxa < 0 and vya > 0:
            #     vxa = vxa * -1

            lefty = int((-x * vy / vx) + y)
            righty = int(((self.image_cols - x) * vy / vx) + y)

            point1 = (self.image_cols - 1, righty)
            point2 = (0, lefty)



            self.points1.append(point1)

            newline = Line((cx, cy), point1, point2, self.image_center)
            newline.angle = degrees(atan2(vya, vxa))# * -1

            if self.heading is not None and -40 < self.heading - newline.angle < 40:
                lines.append(newline)
            else:
                bad_lines.append(newline)

        self.bad_lines = bad_lines
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
        self.roll_drift = -round(self.h.distance_non_abs(cross_point, self.image_center), 0)

    def calculate_yaw_drift(self):
        lines = self.lines
        # lines_ahead = self.lines
        lines_ahead = [l for l in lines if l.guidepoint[1] <= self.image_center[1]]
        lines_behind = [l for l in lines if l.guidepoint[1] > self.image_center[1]]
        self.lines_ahead = lines_ahead

        if len(lines_ahead) < 1 or len(lines_behind) < 1:
            # print("No ahead lines found, nothing todo here ")
            self.yaw_drift = 0
            self.best_course = None
            return

        lines_ahead.sort(key=lambda l: l.centerdistance)

        fstart = (-lines_behind[0].guidepoint[1], lines_behind[0].guidepoint[0])
        fend = (-lines_ahead[0].guidepoint[1], lines_ahead[0].guidepoint[0])
        xDiff = fend[0] - fstart[0]
        yDiff = fend[1] - fstart[1]

        self.correct_yaw = round(degrees(atan2(yDiff, xDiff)), 0)

        yaw_drift = round(self.correct_yaw - self.heading, 0)
        self.yaw_drift = yaw_drift

    def calculate_meters_in_view(self):
        self.meters_front = self.altitude * math.atan(85 / 2)
        # self.pixels_per_m = self.image_rows / 2 / self.meters_front

    def draw_image(self, frame):
        cv2.circle(frame, self.image_center, 7, (200, 100, 255), -1)  # Image center point
        try:
            cv2.drawContours(frame, self.conts[1], -1, (255, 0, 0), 1)
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
            cv2.putText(frame, "heading " + str(round(self.heading + self.yaw, 3)) + " deg", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
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

        # try:
        #     cv2.putText(frame, "north " + str(self.north[-1]), (10, 190),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5,
        #                 (255, 255, 100), 2)
        # except:
        #     pass
        #
        # try:
        #     cv2.putText(frame, "east " + str(self.east[-1]), (10, 210),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5,
        #                 (255, 255, 100), 2)
        # except:
        #     pass

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

        for l in self.lines:
            try:
                cv2.line(frame, l.p1, l.p2, (255, 0, 0), 1)
                cv2.putText(frame, str(round(l.angle, 0)), l.guidepoint, FONT, 0.5,
                            (0, 0, 255), 2)
            except:
                pass

        for l in self.bad_lines:
            try:
                cv2.line(frame, l.p1, l.p2, (0, 0, 255), 1)
                cv2.putText(frame, str(round(l.angle, 0)), l.guidepoint, FONT, 0.5,
                            (0, 0, 255), 2)
            except:
                pass

        time_now = time.time()
        print("TIME", str(round(time_now - self.flight_params_time, 4)))
        cv2.putText(frame, "Time " + str(round(time_now - self.flight_params_time, 4)) + " s", (10, 300),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 100), 2)

        cv2.putText(frame, "Correct Yaw " + str(self.correct_yaw) + " deg", (10, 320), FONT, 0.5, (255, 255, 100), 2)
        cv2.putText(frame, "Yaw drift " + str(self.yaw_drift) + " deg", (10, 340), FONT, 0.5, (255, 255, 100), 2)
        cv2.putText(frame, "Roll drift " + str(self.roll_drift) + " px", (10, 360), FONT, 0.5, (255, 255, 100), 2)

        try:
            cv2.putText(frame, str(round(self.heading, 0)), self.image_center,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 2)
        except:
            pass
        cv2.imwrite("./images/image" + str(time.time()) + ".jpg", frame)



if __name__=="__main__":
    e = Eyes()
    e.start_capture()