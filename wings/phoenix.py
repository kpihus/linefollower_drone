from dronekit import connect, Command, LocationGlobal, VehicleMode
from simple_pid import PID
from models.data import FlightData
from pymavlink import mavutil
import time, sys, argparse, math, threading

TARGET_ALTITUDE = 1.5
UPDATE_INTERVAL = 0.01

def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
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


class Phoenix:
    def __init__(self, flight_params, flight_commands):
        self.connection_string = '127.0.0.1:14540'
        self.vehicle = None
        self.last_heartbeat = None
        self.fpq = flight_params
        self.fcq = flight_commands
        self.last_flight_commands = time.time()
        self.flightData = FlightData()
        self.roll_angle = 0.0
        self.pitch_angle = 0.0
        self.yaw_angle = 0.0
        self.thrust = 0.0
        self.yaw_drift = 0.0
        self.roll_drift = 0.0

    def loop(self):
        self.connect()
        self.vehicle.mode = VehicleMode("LOITER")

        while not self.vehicle.armed:
            print(" Waiting for arming..." + str(self.vehicle.armed))
            self.vehicle.armed = True
            time.sleep(1)

        self.vehicle.mode = VehicleMode("OFFBOARD")
        print("Starting infinite loop")
        while self.vehicle.mode != "LAND":
            self.roll_angle = -1
            self.pitch_angle = -1
            self.yaw_angle = -1
            if not self.fcq.empty():
                flight_commands = self.fcq.get()
                print("Got flight commands")
                print(str(flight_commands))
                if flight_commands and flight_commands.timestamp > self.last_flight_commands:
                    self.yaw_drift = flight_commands.yaw_drift
                    self.roll_drift = flight_commands.roll_drift

            self.gather_info()

            self.altitude_holder()
            self.yaw_holder()
            self.roll_holder()

            self.set_attitude(self.roll_angle, self.pitch_angle, self.yaw_angle, 0.0, False)
            time.sleep(UPDATE_INTERVAL)

    def connect(self):
        print("Connecting to Bird")
        vehicle = connect(self.connection_string, wait_ready=True)
        print(" Type: %s" % vehicle._vehicle_type)
        print(" Armed: %s" % vehicle.armed)
        print(" System status: %s" % vehicle.system_status.state)
        self.vehicle = vehicle

    def gather_info(self):
        altitude = self.vehicle.location.local_frame.down * -1
        pitch = math.degrees(self.vehicle.attitude.pitch)
        roll = math.degrees(self.vehicle.attitude.roll)
        yaw = math.degrees(self.vehicle.attitude.yaw)
        north = self.vehicle.location.local_frame.north
        east = self.vehicle.location.local_frame.east
        speed = self.vehicle.groundspeed
        self.flightData = FlightData(time.time(), altitude, pitch, roll, yaw, north, east, speed)
        self.fpq.put(self.flightData)

    def altitude_holder(self):
        vehicle = self.vehicle

        # Setup PID parameters (NOTE: These are tested values, do not randomly change these)
        kp = 0.35
        ki = 0.2
        kd = 0.35

        # Setup PID
        pid = PID(kp, ki, kd, setpoint=TARGET_ALTITUDE)
        pid.sample_time = UPDATE_INTERVAL # we're currently updating this 0.01
        pid.output_limits = (0, 1)  # thrust can only be 0 to 1

        self.thrust = pid(self.flightData.altitude)

    def roll_holder(self):
        kp = 0.35
        ki = 0.2
        kd = 0.35
        drift = self.roll_drift

    def yaw_holder(self):
        kp = 0.35
        ki = 0.2
        kd = 0.35
        drift = self.yaw_drift

    def land(self):
        vehicle = self.vehicle
        vehicle.mode = VehicleMode("LAND")

    def set_attitude(self, roll_angle=0.0, pitch_angle=0.0,
                     yaw_angle=None, yaw_rate=0.0, use_yaw_rate=True):
        """
        Note that from AC3.3 the message should be re-sent more often than every
        second, as an ATTITUDE_TARGET order has a timeout of 1s.
        In AC3.2.1 and earlier the specified attitude persists until it is canceled.
        The code below should work on either version.
        Sending the message multiple times is the recommended way.
        """

        # if any of the angles are defined as -1, keep whatever vehicle currently has
        current_attitude = self.vehicle.attitude
        roll_angle = current_attitude.roll if roll_angle == -1 else roll_angle
        pitch_angle = current_attitude.pitch if pitch_angle == -1 else pitch_angle
        yaw_angle = current_attitude.yaw if yaw_angle == -1 else yaw_angle

        self.send_attitude_target(roll_angle, pitch_angle, yaw_angle, yaw_rate, use_yaw_rate)

    def send_attitude_target(self, roll_angle=0.0, pitch_angle=0.0,
                             yaw_angle=None, yaw_rate=0.0, use_yaw_rate=True):
        vehicle = self.vehicle
        """
        use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                      When one is used, the other is ignored by Ardupilot.
        thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
                Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
                the code for maintaining current altitude.
        """

        if not use_yaw_rate and yaw_angle is None:
            yaw_angle = vehicle.attitude.yaw

        if yaw_angle is None:
            yaw_angle = 0.0

        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        msg = vehicle.message_factory.set_attitude_target_encode(
            0,  # time_boot_ms
            1,  # Target system
            1,  # Target component
            0b00000000 if use_yaw_rate else 0b00000100,
            to_quaternion(roll_angle, pitch_angle, yaw_angle),  # Quaternion
            0,  # Body roll rate in radian
            0,  # Body pitch rate in radian
            math.radians(yaw_rate),  # Body yaw rate in radian/second
            self.thrust  # Thrust
        )
        vehicle.send_mavlink(msg)