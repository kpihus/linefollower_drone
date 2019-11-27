import cv2
import math
import time
import numpy as np
from dronekit import connect, Command, LocationGlobal, VehicleMode
from functools import reduce
from simple_pid import PID


class OpticalFlow:
    def __init__(self):
        self.vehicle = None
        self.CONNECTION_STRING = '127.0.0.1:14540'
        self.cap = None
        self.UPDATE_INTERVAL = 0.01
        self.TARGET_ALTITUDE = 1.5

        self.THRUST = 0.5
        self.roll_kpid = dict(kp=1, ki=0, kd=1)
        self.pitch_kpid = dict(kp=1, ki=0, kd=1)
        self.thrust_kpid = dict(kp=0.35, ki=0.2, kd=0.35)

        self.thrust_pid = None
        self.roll_pid = None
        self.pitch_pid = None

        self.frame_width = 0
        self.frame_height = 0

        self.lk_params = dict(winSize=(15, 15), maxLevel=2,
                         criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        self.feature_params = dict(maxCorners=100,
                              qualityLevel=0.3,
                              minDistance=7,
                              blockSize=7)

        self.current_roll = 0
        self.current_pitch = 0

        self.roll_angle = 0
        self.pitch_angle = 0
        self.yaw_angle = -999


    def setup_pid(self):
        # Setup PID
        self.thrust_pid = PID(self.thrust_kpid['kp'], self.thrust_kpid['ki'], self.thrust_kpid['kd'], setpoint=self.TARGET_ALTITUDE)
        self.thrust_pid.sample_time = self.UPDATE_INTERVAL  # we're currently updating this 0.01
        self.thrust_pid.output_limits = (0, 1)

        # PID for roll
        self.roll_pid = PID(self.roll_kpid['kp'], self.roll_kpid['ki'], self.roll_kpid['kd'], setpoint=0)
        self.roll_pid.sample_time = self.UPDATE_INTERVAL  # we're currently updating this 0.01
        self.roll_pid.output_limits = (-2, 2)  # roll angle should only be between 5 / -5

        # PID for roll
        self.pitch_pid = PID(self.pitch_kpid['kp'], self.pitch_kpid['ki'], self.pitch_kpid['kd'], setpoint=0)
        self.pitch_pid.sample_time = self.UPDATE_INTERVAL  # we're currently updating this 0.01
        self.pitch_pid.output_limits = (-2, 2)  # pitch angle should only be between 5 / -5


    def connect(self):
        self.vehicle = connect(self.CONNECTION_STRING, wait_ready=False, )
        self.vehicle.wait_ready('system_status', 'mode', 'armed', 'attitude')

        print(" Type: %s" % self.vehicle._vehicle_type)
        print(" Armed: %s" % self.vehicle.armed)
        print(" System status: %s" % self.vehicle.system_status.state)
        while not self.vehicle.armed:
            self.gather_info()
            print(" Waiting for arming..." + str(self.vehicle.armed))
            self.vehicle.armed = True
            time.sleep(1)


    def gather_info(self):
        self.pitch_angle = math.degrees(self.vehicle.attitude.pitch)
        self.roll_angle = math.degrees(self.vehicle.attitude.roll)
        # print("RP", roll, pitch)


    def altitude_holder(self):
        self.THRUST = self.thrust_pid(self.vehicle.location.local_frame.down * -1)

    def to_quaternion(self, roll=0.0, pitch=0.0, yaw=0.0):
        """
        Convert degrees to quaternions
        """
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5

        return [w, x, y, z]


    def set_attitude(self, roll_angle=0.0, pitch_angle=0.0,
                     yaw_angle=None, yaw_rate=0.0, use_yaw_rate=True):

        # if any of the angles are defined as -1, keep whatever vehicle currently has
        current_attitude = self.vehicle.attitude
        roll_angle = math.degrees(current_attitude.roll) if roll_angle == -999 else roll_angle
        pitch_angle = math.degrees(current_attitude.pitch) if pitch_angle == -999 else pitch_angle
        yaw_angle = math.degrees(current_attitude.yaw) if yaw_angle == -999 else yaw_angle

        self.send_attitude_target(roll_angle, pitch_angle, yaw_angle, yaw_rate, use_yaw_rate)


    def send_attitude_target(self, roll_angle=0.0, pitch_angle=0.0,
                             yaw_angle=None, yaw_rate=0.0, use_yaw_rate=True):

        if not use_yaw_rate and yaw_angle is None:
            yaw_angle = self.vehicle.attitude.yaw

        if yaw_angle is None:
            yaw_angle = 0.0

        print("New attitude", roll_angle, pitch_angle, yaw_angle, self.THRUST)

        msg = self.vehicle.message_factory.set_attitude_target_encode(
            0,  # time_boot_ms
            1,  # Target system
            1,  # Target component
            0b00000000 if use_yaw_rate else 0b00000100,
            self.to_quaternion(roll_angle, pitch_angle, yaw_angle),  # Quaternion
            0,  # Body roll rate in radian
            0,  # Body pitch rate in radian
            math.radians(yaw_rate),  # Body yaw rate in radian/second
            self.THRUST  # Thrust
        )
        self.vehicle.send_mavlink(msg)

    def start_capture(self):
        capture_src = 'udpsrc buffer-size=24000000 port=5600 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)RAW, sampling=YCbCr-4:2:0,depth=(string)16,width=(string)640, height=(string)480,colorimetry=(string)BT601-5, payload=(int)96, a-framerate=60/1" ! rtpvrawdepay ! videoconvert ! queue ! appsink drop=true'
        capture_opts = cv2.CAP_GSTREAMER

        self.cap = cv2.VideoCapture(capture_src, capture_opts)

        if not self.cap.isOpened():
            print("VideoCapture not opened")
            exit(-1)
        print("Cap is opened")

        self.frame_width = int(self.cap.get(3))
        self.frame_height = int(self.cap.get(4))


    def initialize_optflow(self):
        print("Starting optflow")
        # params for ShiTomasi corner detection

        # Parameters for lucas kanade optical flow


        # Take first frame and find corners in it
        ret, frame = self.cap.read()
        old_frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
        p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **self.feature_params)
        # Create a mask image for drawing purposes
        mask = np.zeros_like(old_frame)
        self.opt_flow(old_gray, mask, p0)

    def opt_flow(self, old_gray, mask, p0):
        print("Going optflow ...")
        color = np.random.randint(0, 255, (100, 3))

        self.set_attitude(self.roll_angle, self.pitch_angle, self.yaw_angle, 0.0, False)
        # self.vehicle.mode = VehicleMode("OFFBOARD")

        ROLL_DRIFT_SUM = 0
        PITCH_DRIFT_SUM =0
        while True:
            self.gather_info()
            self.altitude_holder()
            self.set_attitude(self.roll_angle, self.pitch_angle, self.yaw_angle, 0.0, False)
            pitch_drifts = []
            roll_drifts = []
            self.cap.grab()
            ret, image = self.cap.retrieve()
            if not ret:
                print('frame empty')
                continue
            # rotate image 90 deg, because of landscape camera on drone
            frame = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if p0 is None or len(p0) < 5:
                print("Empty p0")
                mask = np.zeros_like(frame)
                p0 = cv2.goodFeaturesToTrack(frame_gray.copy(), mask=None, **self.feature_params)
                ROLL_DRIFT_SUM = 0
                PITCH_DRIFT_SUM = 0
                continue
            # calculate optical flow
            p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **self.lk_params)
            # Select good points
            if p1 is None:
                print("URROR")
                old_frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
                p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **self.feature_params)
                ROLL_DRIFT_SUM = 0
                PITCH_DRIFT_SUM = 0
                continue
            good_new = p1[st == 1]
            good_old = p0[st == 1]
            # draw the tracks
            for i, (new, old) in enumerate(zip(good_new, good_old)):
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

            self.roll_angle = self.roll_pid(ROLL_DRIFT_SUM)
            self.pitch_angle = self.pitch_pid(PITCH_DRIFT_SUM)

            self.set_attitude(self.roll_angle, self.pitch_angle, self.yaw_angle, 0.0, False)
            # print("Roll/Pitch", ROLL_DRIFT, PITCH_DRIFT)
            img = cv2.add(frame, mask)
            cv2.imshow('frame', img)
            # out.write(img)
            k = cv2.waitKey(30) & 0xff
            if k == 27:
                break
            # Now update the previous frame and previous points
            old_gray = frame_gray.copy()
            p0 = good_new.reshape(-1, 1, 2)

        cv2.destroyAllWindows()


o = OpticalFlow()
o.setup_pid()
o.connect()
o.start_capture()
o.initialize_optflow()