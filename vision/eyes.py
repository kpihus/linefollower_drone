import os

if os.getenv('PLATFORM') == 'BIRD':
    from picamera.array import PiRGBArray
    from picamera import PiCamera

import cv2
import math
import time

import numpy as np
from math import atan2, degrees, tan
import base64

from helpers.vision import Helpers
from vision.shape import Line
from vision.shapedetector import ShapeDetector
from models.data import FlightCommands
from functools import reduce

MIN_AREA = 300
MAX_AREA = 4500
MIN_RATIO = 0.10
MAX_RATIO = 0.25

FONT = cv2.FONT_HERSHEY_SIMPLEX


class Eyes:
    def __init__(self, flight_params=None, flight_commands=None, image_queue=None):
        self.start_time = time.time()
        self.image_queue = image_queue
        self.capture_src = 'udpsrc buffer-size=24000000 port=5600 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)RAW, sampling=YCbCr-4:2:0,depth=(string)16,width=(string)640, height=(string)640,colorimetry=(string)BT601-5, payload=(int)96, a-framerate=60/1" ! rtpvrawdepay ! videoconvert ! queue ! appsink drop=true'
        # self.capture_src = 'udpsrc port="5600" caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)RAW, sampling=(string)YCbCr-4:2:0, depth=(string)8, width=(string)720, height=(string)1280, colorimetry=(string)BT601-5, payload=(int)96, ssrc=(uint)1103043224, timestamp-offset=(uint)1948293153, seqnum-offset=(uint)27904" ! rtpvrawdepay ! videoconvert ! queue ! appsink sync=false'
        self.capture_opts = cv2.CAP_GSTREAMER
        self.state = 'INIT'
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
        self.lines = []
        self.bad_lines = None
        self.lines_ahead = None
        self.roll_line = None
        self.line_length = 0
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
        self.opt_flow_speed_history = []

        self.lk_params = dict(winSize=(15, 15), maxLevel=2,
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        self.feature_params = dict(maxCorners=100,
                                   qualityLevel=0.3,
                                   minDistance=7,
                                   blockSize=7)

        self.opt_flow_speed = 0

        self.north = []
        self.east = []
        self.speed = 0
        self.altitude = 0
        self.meters_front = 0
        self.pixels_per_m = 0
        self.velocity_line = None
        self.moving_line = None
        self.drive_dir = None
        self.last_non_zero_roll_drift = 0

        self.points1 = []
        self.number_of_good_lines = 0
        self.old_opt_flow_frame = None
        self.p0 = None

        self.camera = None
        self.raw_capture = None

    def start_capture(self):
        # print('The beginning')
        if os.getenv('PLATFORM') == 'SIMU':
            self.capture_simu()
        elif os.getenv('PLATFORM') == 'BIRD':
            self.initialize_camera()
            self.capture_frame()
            pass
        else:
            # print('Unknown platform')
            exit()

    def initialize_camera(self):
        camera = PiCamera()
        camera.resolution = (640, 480)
        camera.framerate = 32
        raw_capture = PiRGBArray(camera)
        time.sleep(0.3)
        self.camera = camera
        self.raw_capture = raw_capture

    def capture_simu(self):
        # print("Starting video capture")
        self.cap = cv2.VideoCapture(self.capture_src, self.capture_opts)
        # print("got cap")
        if not self.cap.isOpened():
            # print("VideoCapture not opened")
            exit(-1)
        # print("Cap is opened")

        while True:
            # print("Got frame")
            self.flight_info()

            self.cap.grab()

            ret, image = self.cap.retrieve()
            if not ret:
                # print('frame empty')
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
                self.state = data.state
                # self.thres_val = self.h.map_values(data.thres_val, inMin=1000, inMax=2000, outMin=0, outMax=255) \
                #     if not (data.thres_val is None) and data.thres_val > 0 else self.thres_val
                # self.thres_max = self.h.map_values(data.thres_max, inMin=1000, inMax=2000, outMin=0, outMax=255) \
                #     if not (data.thres_max is None) and data.thres_max > 0 else self.thres_max

                # update location only when moving
                if self.speed >= 0.1:
                    self.north.append(data.north)
                    self.east.append(data.east)

    def capture_frame(self):
        for image in self.camera.capture_continuous(self.raw_capture, format="bgr", use_video_port=True):
            self.start_time = time.time()
            self.flight_info()
            frame = image.array
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            self.process_frame(frame)
            self.raw_capture.truncate(0)

    def process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        opt_gray = gray.copy()
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.threshold(blurred, self.thres_val, self.thres_max, cv2.THRESH_BINARY)[1]
        self.conts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        rows, cols = frame.shape[:2]  # frame dimensions
        self.image_cols = cols
        self.image_rows = rows

        self.image_center = (int(cols / 2), int(rows / 2))

        self.heading = 0  # angle - self.yaw
        if self.heading < -180:
            self.heading += 360

        # Calculate and process all kind of stuff
        #self.opt_flow(opt_gray)
        self.process_contours(frame)
        self.calculate_roll_drift()
        self.calculate_yaw_drift()
        self.calculate_meters_in_view()


        if self.roll_drift != 0:
            self.last_non_zero_roll_drift = self.roll_drift

        self.fcq.put(
            FlightCommands(time.time(),
                           self.yaw_drift,
                           self.last_non_zero_roll_drift,
                           self.number_of_good_lines,
                           self.opt_flow_speed))
        # self.draw_image(frame)

    def process_contours(self, frame):
        lines = []
        bad_lines = []
        # dont process frames with huge amount of contours
        if self.conts[1] and len(self.conts[1]) > 50:
            print("Skipping image due to cont count", len(self.conts[1]))
            return
        for c in self.conts[1]:
            # x, y, w, h = cv2.boundingRect(c)
            # cv2.rectangle(oimg, (x, y), (x + w, y + h), (255, 255, 255), 2)
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            area = cv2.contourArea(box)
            # if MIN_AREA < area < MAX_AREA:
            #   continue
            moments = cv2.moments(c)
            if moments["m00"] == 0:
                continue
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            contour_width = rect[1][0]
            contour_height = rect[1][1]
            # calculates contour side ratio
            contour_side_ratio = contour_height / contour_width if contour_height < contour_width else contour_width / contour_height
            [vx, vy, x, y] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
            # Point all line to northish direction
            vxa = -vy
            vya = vx
            if vxa < 0:
                vxa = -vxa
                vya = -vya
            lefty = int((-x * vy / vx) + y)
            righty = int(((self.image_cols - x) * vy / vx) + y)
            point1 = (self.image_cols - 1, righty)
            point2 = (0, lefty)
            self.points1.append(point1)
            newline = Line((cx, cy), point1, point2, self.image_center)
            newline.angle = degrees(atan2(vya, vxa))  # * -1
            if MIN_AREA < area < MAX_AREA and MIN_RATIO < contour_side_ratio < MAX_RATIO and -40 < self.heading - newline.angle < 40:
                lines.append(newline)
            else:
                bad_lines.append(newline)
            # if rect[1][0] > 10:
            # cv2.putText(frame,
            #             str(round(rect[1][0])) + '-' + str(round(rect[1][1])) + '|' + str(area) + ' | ' + str(
            #                 round(contour_side_ratio, 2)), (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            #             (10, 100, 100), 2)

        self.bad_lines = bad_lines
        self.lines = lines
        self.number_of_good_lines = len(lines)

    def opt_flow(self, frame):
        mask = np.zeros_like(frame)

        if self.old_opt_flow_frame is None:
            print("Saving old opt flow frame")
            self.old_opt_flow_frame = frame
            # find good features from last frame
            self.p0 = cv2.goodFeaturesToTrack(self.old_opt_flow_frame, mask=None, **self.feature_params)
            return

        ROLL_DRIFT_SUM = 0
        PITCH_DRIFT_SUM = 0
        pitch_drifts = []
        roll_drifts = []

        if self.p0 is None or len(self.p0) < 5:
            print("Empty p0, no features to find from last frame")
            self.old_opt_flow_frame = frame
            self.p0 = cv2.goodFeaturesToTrack(self.old_opt_flow_frame, mask=None, **self.feature_params)
            return
        # calculate optical flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_opt_flow_frame, frame, self.p0, None, **self.lk_params)
        # Select good points
        if p1 is None:
            print("No optical flow drift found, p1 empty")
            return

        good_new = p1[st == 1]
        good_old = self.p0[st == 1]
        # draw the tracks
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            color = np.random.randint(0, 255, (100, 3))
            a, b = new.ravel()
            c, d = old.ravel()
            mask = cv2.line(mask, (a, b), (c, d), color[i].tolist(), 2)
            frame = cv2.circle(frame, (a, b), 5, color[i].tolist(), -1)
            roll_drifts.append(c - a)
            pitch_drifts.append(d - b)

        if len(pitch_drifts) < 1 or len(roll_drifts) < 1:
            print('No drift points')
        elif len(pitch_drifts) < 2 or len(roll_drifts) < 2:
            ROLL_DRIFT_SUM += roll_drifts[0]
            PITCH_DRIFT_SUM += pitch_drifts[0]
        else:
            ROLL_DRIFT_SUM += reduce(lambda a, b: a + b, roll_drifts) / len(roll_drifts)
            PITCH_DRIFT_SUM += reduce(lambda a, b: a + b, pitch_drifts) / len(pitch_drifts)

        # make this as last opt flow frame
        self.old_opt_flow_frame = frame

        if len(self.opt_flow_speed_history) >= 10:
            self.opt_flow_speed_history.pop(0)

        print("Roll_DRIFT_SUM", ROLL_DRIFT_SUM)
        self.opt_flow_speed_history.append((time.time(), ROLL_DRIFT_SUM))

        first_point = self.opt_flow_speed_history[0]
        last_point = self.opt_flow_speed_history[-1]
        distance_traveled = last_point[1] - first_point[1]
        time_traveled = last_point[0] - first_point[0]

        self.p0 = good_new.reshape(-1, 1, 2)

        if time_traveled == 0:
            return

        self.opt_flow_speed = distance_traveled / time_traveled
        img = cv2.add(frame, mask)
        self.send_image(img)
        # cv2.destroyAllWindows()

    def calculate_roll_drift(self):
        lines = self.lines
        if not self.lines or len(lines) < 2:
            self.roll_line = None
            self.roll_drift = 0
            return
        lines.sort(key=lambda l: l.centerdistance)
        roll_line = Line(lines[0].guidepoint, lines[0].guidepoint, lines[1].guidepoint, self.image_center)
        cross_line = roll_line.plot_point(self.image_center, int(roll_line.angle) + 90, 250)
        cross_point = Line.line_intersection((roll_line.p1, roll_line.p2), (cross_line[0], cross_line[1]))
        self.roll_line = roll_line
        self.roll_drift = -round(self.h.distance_non_abs(cross_point, self.image_center), 0)
        # if self.yaw_drift < -10 or self.yaw_drift > 10:
        #    self.roll_drift = 0

    def calculate_yaw_drift(self):
        lines = self.lines
        if not self.lines:
            return
        # lines_ahead = self.lines
        lines_ahead = [l for l in lines if l.guidepoint[1] <= self.image_center[1]]
        lines_behind = [l for l in lines if l.guidepoint[1] > self.image_center[1]]
        self.lines_ahead = lines_ahead
        if len(lines_ahead) < 1:
            # print("No ahead lines found, nothing todo here ")
            self.yaw_drift = 0
            self.best_course = None
            return
        lines_ahead.sort(key=lambda l: l.centerdistance)
        # fstart = (-lines_behind[0].guidepoint[1], lines_behind[0].guidepoint[0])
        # fend = (-lines_ahead[0].guidepoint[1], lines_ahead[0].guidepoint[0])
        # xDiff = fend[0] - fstart[0]
        # yDiff = fend[1] - fstart[1]
        # self.correct_yaw = round(degrees(atan2(yDiff, xDiff)), 0)
        # yaw_drift = round(self.correct_yaw - self.heading, 0)
        yaw_drift = lines_ahead[0].angle
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
            cv2.putText(frame, "heading " + str(round(self.heading + self.yaw, 3)) + " deg", (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
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
        try:
            cv2.putText(frame, "Length " + str(self.line_length), (10, 210),
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

        if self.lines:
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
        self.send_image(frame)

    def send_image(self, frame):
        to_send = cv2.resize(frame, (600, 480))

        encoded, buffer = cv2.imencode('.jpg', to_send)
        jpg_as_text = base64.b64encode(buffer)
        self.image_queue.put(jpg_as_text)


if __name__ == "__main__":
    e = Eyes()
    e.start_capture()
